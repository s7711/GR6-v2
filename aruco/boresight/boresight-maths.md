# Bore-sight maths

A human-readable account of what `model.py`/`solve.py` actually compute.
Code stays the authority — this is the derivation behind it, so a
formula here should always match a line there; if they ever disagree,
the code is right and this needs fixing. GitHub renders the `$...$` /
`$$...$$` below as maths; a plain-text viewer will show the LaTeX source
instead, which is still readable.

## 1. What this is

A camera bolted to the robot sees ArUco markers. The xNAV650 INS reports
where the robot is and which way it's pointing. If we knew the camera's
mounting angle relative to the vehicle — $\text{hpr}_{cb}$, three numbers
— and the markers' exact positions, we could *predict* exactly where
each marker's corners land in the image. We don't know either, so this
is a **bundle adjustment**: find the $\text{hpr}_{cb}$ and marker poses
that make the predicted corners match the *detected* corners as closely
as possible, over every frame of a driven session at once.

The reason to work in raw pixels rather than ArUco's own per-frame pose
estimate (`rvec`/`tvec`) is weighting. A corner's *bearing* from the
camera is good to about 0.01° (the calibration file's own RMS
reprojection error is 0.18px, and $\text{fx}\approx1166\text{px}$ gives
$0.18/1166 \approx 0.01°$ per pixel). A small planar marker's *out-of-plane
rotation*, recovered from `rvec`, is a textbook ill-conditioned
problem and can be degrees out from a single detection. Minimising pixel
error automatically weights every corner by how well it's actually
known; hand-picking weights for an `rvec`/`tvec` residual would not.

## 2. Frames

Six frames, all from `coords.py` — nothing here is a second convention:

| frame | name | axes | fixed by |
|---|---|---|---|
| $n$ | nav | X north, Y east, Z down | — |
| $b$ | body | X forward, Y right, Z down | the vehicle |
| $C$ | raw camera | X right, Y down, Z along boresight | OpenCV's convention |
| $c$ | HPR-friendly camera | X forward, Y right, Z down | chosen so the mount angle is an ordinary HPR triple |
| $M$ | raw marker | X right, Y up, Z out of the printed face | `cv2.aruco`'s convention |
| $m$ | HPR-friendly marker | X out the *back*, Y right, Z down | chosen so a marker's pose is an ordinary HPR triple |

$c$ and $m$ only exist so that "camera mount angle" and "marker
orientation" can each be written as a familiar heading/pitch/roll
triple, the same way vehicle attitude is. $C$ and $M$ are what OpenCV
actually hands back or expects. Converting between the two conventions
for the same physical object is a **fixed relabelling**, not a
measurement — e.g. $X_m$ (out the marker's back) $= -Z_M$ (out the
marker's front).

## 3. The rotation chain

Every DCM here has the same form, built once and reused for vehicle
attitude, marker orientation, and the camera mount alike:

$$
C_{nb}(h,p,r) = R_z(h)\,R_y(p)\,R_x(r)
$$

read as "$C_{nb}$ rotates a vector's components from frame $b$ into
frame $n$": $V_n = C_{nb} V_b$. The full chain from body to nav, via the
camera and the marker (read right to left):

$$
C_{nb} = C_{nm}\,C_{mM}\,C_{MC}\,C_{Cc}\,C_{cb}
$$

$$
V_b \xrightarrow{C_{cb}} V_c \xrightarrow{C_{Cc}} V_C \xrightarrow{C_{MC}} V_M \xrightarrow{C_{mM}} V_m \xrightarrow{C_{nm}} V_n
$$

- $C_{cb} = C_{nb}(\text{hpr}_{cb})$ — the camera mount angle. **This is
  the unknown being solved for.**
- $C_{Cc}$, $C_{mM}$ — fixed, definitional axis relabellings (constant
  matrices in `coords.py`).
- $C_{MC} = C_{CM}^{\mathsf T}$, and $C_{CM} = \text{Rodrigues}(\text{rvec})$
  — the one live measurement in the whole chain, straight from
  `cv2.aruco.estimatePoseSingleMarkers` (though the solve here never
  actually calls that on the real data — see §4, it re-derives the
  equivalent information from raw corner pixels instead).
- $C_{nm} = C_{nb}(\text{marker\_hpr})$ — a marker's orientation. **Also
  an unknown**, one free triple per marker.

## 4. Forward projection

Given a candidate $\text{hpr}_{cb}$, a marker's pose
$(\text{pos}_n, \text{hpr}_m)$, and the vehicle's reported pose
$(\text{nav\_pos}_n, \text{nav\_hpr})$, `model.py` predicts where the
marker's four corners land in the image. The corners in the marker's
own frame $M$ are just its four physical corners:

$$
X_M \in \left\{ \left(\pm\tfrac{s}{2}, \pm\tfrac{s}{2}, 0\right) \right\}, \quad s = \text{marker size}
$$

Chained forward to the raw camera frame:

$$
\begin{aligned}
X_m &= C_{mM}\,X_M \\
X_n &= \text{pos}_n + C_{nm}\,X_m \\
X_b &= C_{nb}^{\mathsf T}\,(X_n - \text{nav\_pos}_n) \\
X_{b,\text{cam}} &= X_b - d_{xc\_b} \\
X_C &= C_{Cc}\,C_{cb}\,X_{b,\text{cam}}
\end{aligned}
$$

$d_{xc\_b}$ is the camera's fixed position offset from the body origin
(CAD, ~5mm, held constant — see §8). Then OpenCV's own pinhole +
distortion model turns $X_C$ into a pixel:

$$
u = \pi(X_C;\ K,\ \text{dist})
$$

where $K$ is the 3×3 camera matrix and $\pi$ is `cv2.projectPoints` —
radial + tangential distortion applied after the standard pinhole
divide. This whole computation is done for every corner of every
observation at once (`model.project`, one batched call rather than one
per detection).

## 5. Unknowns

The parameter vector, for $N_{\text{markers}}$ markers:

$$
\theta = \big(\underbrace{\text{hpr}_{cb}}_{3},\ \underbrace{\text{pos}_1,\ldots,\text{pos}_{N}}_{3N},\ \underbrace{\text{hpr}_1,\ldots,\text{hpr}_{N}}_{3N}\big)
\quad\left(+\ \underbrace{\sigma_{\text{size}}}_{1}\text{, optionally}\right)
$$

Three markers gives $3 + 18 = 21$ unknowns (or 22 with the size scale).

**Why each marker's full 6-DOF pose is free** rather than surveyed or
assumed: Ben can plant a marker only approximately, and can't measure
inter-marker spacing to better than a centimetre or two by hand. Making
every marker pose a nuisance parameter costs 18 extra unknowns but buys
total independence from placement accuracy — the markers get surveyed
as a *by-product* of the calibration, more accurately than they were
planted.

**Why an optional global size scale $\sigma_{\text{size}}$**: not
because a wrong assumed size would bias $\text{hpr}_{cb}$ — it measurably
doesn't (§9) — but because it turns the calibration run into an
independent check on the hand-measured marker size, which matters
elsewhere (the live single-shot GAD path uses `tvec` directly, and
*that* scales linearly with assumed size).

## 6. Cost function

Plain nonlinear least squares over every corner of every observation,
in pixel space, one term per corner coordinate:

$$
J(\theta) = \sum_{i=1}^{N_{\text{obs}}} \sum_{k=1}^{4} \left\lVert \frac{u_{i,k}(\theta) - \hat u_{i,k}}{\sigma_{px}} \right\rVert^2
$$

where $u_{i,k}(\theta)$ is the predicted pixel for corner $k$ of
observation $i$ (§4, a function of $\theta$ through $\text{hpr}_{cb}$
and that observation's marker pose), $\hat u_{i,k}$ is the *detected*
pixel, and $\sigma_{px} = 0.2$px is the assumed per-coordinate corner
noise (close to the calibration file's own 0.18px RMS). `residuals()`
returns the flattened, sigma-normalised residual vector; `scipy.optimize
.least_squares` (`method="trf"`, trust-region-reflective) minimises the
sum of its squares. A real capture optionally adds a Huber loss —
scipy's convention, $\rho(z) = z$ for $z \le 1$ and $\rho(z) = 2\sqrt z - 1$
beyond it, applied to $z = (r_i/f_{\text{scale}})^2$ — so residuals
past the threshold grow linearly rather than quadratically, and one
badly mis-detected or half-occluded corner can't drag the whole answer.
Simulated data, being clean by construction, doesn't need it and skips
it to keep the covariance interpretation in §8 simple.

Every corner of every marker in every frame goes into the *same* least
squares problem simultaneously — there's no per-frame or per-marker
solve being averaged afterwards. That's what makes the free marker poses
affordable: a marker seen from 400 different viewpoints is a
heavily-overdetermined 6-parameter fit, even though $\text{hpr}_{cb}$
is shared, and constrained, across all of them at once.

## 7. Starting point

Trust-region least squares needs a starting guess in the right basin,
not the right answer. `initial_marker_poses` gets one cheaply: run
ArUco's own single-shot pose estimate
(`estimatePoseSingleMarkers`) on a subsample of each marker's
detections (up to 60, evenly spread through its observations), convert
each to a nav-frame marker pose using the *assumed* (approximately
correct) $\text{hpr}_{cb}$, and take the per-component median. One bad
detection can't set the starting point; small remaining errors don't
matter because the least-squares solve only needs a basin, not a
result — $\text{hpr}_{cb}$ is already assumed close, this only has to land
each marker within its neighbourhood.

## 8. Uncertainty

Two different numbers, answering two different questions, and they can
disagree by an order of magnitude — that's expected, not a bug.

**Formal covariance** (`hpr_cb_sigma_formal`): the standard
linearised-least-squares result,

$$
\text{Cov}(\theta) \approx \left(J^{\mathsf T} J\right)^{-1} \cdot \chi^2_{\text{red}}, \qquad
\chi^2_{\text{red}} = \frac{1}{\text{dof}}\sum_i \left(\frac{r_i}{\sigma_{px}}\right)^2
$$

where $J$ is the Jacobian of the residual vector at the solution (which
`least_squares` already assembles) and $\chi^2_{\text{red}}$ rescales
the assumed $\sigma_{px}$ by however well it actually fit — if the real
corner noise is bigger than assumed (mounting flex, a marker that isn't
perfectly flat, distortion model error near the edges), this notices
and inflates the reported uncertainty accordingly, rather than trusting
$\sigma_{px}=0.2\text{px}$ blindly.

This number reflects **corner noise and geometry only**. It assumes the
$N_{\text{obs}}$ nav poses fed in are *exactly* correct — no INS error
at all — which is wildly optimistic: HeadingAcc is typically ~0.2°
against ~0.01° of camera bearing noise, so the INS is the dominant real
error source and the formal number doesn't see it. Left alone, this
would report an answer that's confidently wrong.

**The honest number** (`split_check`): solve independently on
$n$ contiguous *time* slices of the same session and look at how much
the answer moves between them —

$$
\hat\sigma_{\text{hpr}_{cb}} = \frac{\text{stdev}(\hat{\text{hpr}}_{cb}^{(1)},\ldots,\hat{\text{hpr}}_{cb}^{(n)})}{\sqrt n}
$$

the standard error of the mean across groups. This measures *whatever
is actually varying* — a slowly-drifting INS heading error, mount flex,
anything — without needing a model of it. Splitting must be on real
capture time, not row order (rows arrive grouped by marker as often as
by time) and not interleaved epochs (neighbouring instants are highly
correlated, so interleaving would report a reassuringly tiny scatter
that means nothing — see §10's OU-process discussion in `sim.py`).

## 9. Marker size doesn't bias $\text{hpr}_{cb}$

Worth stating precisely, since it's the opposite of the first intuition.
Scaling every assumed marker size by $s$ scales the *predicted range* to
each corner by $s$, but $\text{hpr}_{cb}$ is fixed entirely by *bearings* —
angular directions to the corners — which a uniform range scale doesn't
change at all. With many viewpoints and RTK-known vehicle positions, the
bearings alone pin each marker's position hard enough that a size error
has nowhere to go but into the marker's own solved position (and the
residual, if the size scale isn't a free parameter) — not into
$\text{hpr}_{cb}$. Measured directly: a 3.3% size error with the scale
held fixed shifted $\text{hpr}_{cb}$ by 0.0035° and every solved marker
position by 0mm (`test_solve.py`). This is *not* true of the live
single-shot GAD path, which uses `tvec` directly — its magnitude scales
linearly with assumed size, with no bundle of independent views to pin
it down instead.

## 10. Why the capture geometry matters — three degeneracies

These are the reasons the capture path looks the way it does (see
`boresight-prd.md`), stated as maths rather than driving instructions.

**Heading is exactly degenerate under rotations alone.** Yaw the camera
mount by $\delta\psi$ and every marker's heading by $-\delta\psi$
simultaneously, and no `rvec` — no *rotation* — changes: the chain in
§3 only ever sees $C_{cb}$ and $C_{nm}$ multiplied together with other
fixed rotations in between, and a common rotation about the shared
vertical axis, split between the two of them in any proportion, leaves
their relative rotation identical. **It's broken by position, not
rotation.** A camera heading error $\delta\psi$ makes the solver
misattribute an apparent sideways shift of about $r\,\delta\psi$ (range
times angle) to the marker's position, in the *body* frame — and a
body-frame error rotates into a *different* nav-frame direction for
every vehicle heading the marker is viewed from:

$$
\Delta X_n(\text{heading } \phi) \approx C_{nb}(\phi)\, \begin{pmatrix}0\\ r\,\delta\psi\\ 0\end{pmatrix}
$$

A single marker position can satisfy this for every $\phi$ simultaneously
only if $\delta\psi = 0$. So **azimuth spread — viewing each marker from
many different vehicle headings — is the single most valuable thing the
capture path provides**, which is exactly what the fan's wide bearing
sector is for.

**Roll is near-degenerate with a marker's own roll when that marker
sits at the image centre.** Camera roll rotates the whole image about
the principal point; a point at pixel radius $\rho$ from centre is
displaced tangentially by approximately

$$
\Delta u \approx \rho\,\delta\phi
$$

At $\rho \approx 0$ this vanishes regardless of $\delta\phi$ — an
on-axis marker constrains roll almost not at all, and the solver is
free to trade a camera roll error against the marker's own solved roll.
It's broken by observations well off-centre, at large $\rho$ — the
`past` legs exist specifically to swing a marker out towards the image
edge for this reason; content near $\rho \approx 600\text{px}$ is worth
several times what the panel's own ~1m baseline provides on its own.

**Pitch trades against marker height as range times angle.** A camera
pitch error $\delta\theta$ shifts the predicted elevation angle to a
marker by $\delta\theta$; the same shift is produced by moving the
marker's assumed height by

$$
\Delta h \approx -r\,\delta\theta
$$

since a marker's vertical angular position is $\approx h/r$ for modest
angles, so $\partial(\text{elevation})/\partial h = 1/r$. A *real*
pitch error needs a height correction that scales with $r$; a *real*
height error needs a constant one. Viewing the same marker from
multiple ranges (the fan's near/far radii) is what tells the solver
which one it actually is.

## 11. INS heading bias is absorbed, on purpose

A constant INS heading bias $b$ is nearly indistinguishable from a
$\text{hpr}_{cb}$ heading error, and the solve absorbs it into
$\text{hpr}_{cb}$ rather than fighting it — correctly, since ArUco's own
GAD updates want the camera consistent with the frame the xNAV650
*reports*, not with true geographic north.

**Why it's nearly exact.** With the camera at the body origin
($d_{xc\_b}=0$) and zero mount angle, on level ground ($p=r=0$):

$$
C_{nb}(h+b,\,0,\,0) = R_z(b)\,C_{nb}(h,\,0,\,0)
$$

— a pure rotation about the shared vertical, which commutes cleanly
with the rest of the chain. Since every marker's own pose is a *free*
parameter, the whole trajectory-plus-markers system can be rotated by
$R_z(b)$ about the vertical and reproduce identical pixels: the bias is
invisible.

**Why it's not exact — two small leaks, both measured:**

- **The 0.105m lever arm.** With $d_{xc\_b} \neq 0$, the camera's actual
  position is $\text{nav\_pos}_n + C_{nb}\,d_{xc\_b}$. A heading bias
  $b$ shifts that position by approximately $b \times (C_{nb}\,d_{xc\_b})$
  — a genuine *translation* of the camera, which no rotation
  ($\text{hpr}_{cb}$ only ever rotates) can reproduce. Measured: ~1.3%
  of the bias leaks through rather than being absorbed.
- **A non-zero mount angle.** Once $C_{cb}\neq I$, a heading rotation
  about the *body* vertical and one about the *camera's own* vertical
  aren't quite the same axis any more, so a little of the bias leaks
  into the solved pitch/roll instead of heading.

Vehicle tilt — pitch/roll on the actual ground, which looks like the
obvious way to break the degeneracy — turns out **not to matter
measurably** (a 0.15° INS bias leaks ~0.002° from this route). The
lever arm is the real reason the numbers don't come out at exactly the
INS's bias.

## 12. The geometry sanity check

None of the above protects against a dataset with no real information
in it. A stationary capture — camera never moving — can fit the pixels
of a single, fixed viewpoint *beautifully* by trading marker distance
against marker size almost for free: one real session solved to a
marker size of 11.25 **metres**, markers "130m" away, $\text{hpr}_{cb}$ at
$[76°, 0.8°, 75°]$, with `success: True` and a 0.42px residual — nothing
in the fit quality hinted at it, because with one viewpoint there's a
whole family of (distance, size, angle) combinations that reproduce the
same image. `geometry_check()` catches this directly and explicitly,
by requiring real position spread (≥1.5m), heading spread (≥25°, circular
so a run either side of north isn't misread as 360°), and per-marker
viewpoint range (≥0.5m) before a solve is trusted at all — independent
of whatever the residual says.
