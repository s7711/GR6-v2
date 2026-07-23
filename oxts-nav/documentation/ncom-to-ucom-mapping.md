# NCOM → UCOM field mapping (oxts-nav decoder, 304 fields)

Legend: **confirmed** = name+description+units cross-checked in UCOM manual. **likely** =
plausible match by name/purpose/position but not fully cross-verified against units/scaling.
**NOT FOUND** = no UCOM equivalent located.

Source abbreviations used below match the UCOM "Sources And Their Signals" IDs, e.g.
`BNS_SDN` (strapdown nav), `BNS_CFG`/`INS_CFG`/`GNSS_CFG` (config), `BNS_OPT`/`BNS_ACCOPT`
(optimised antenna/lever-arm params + their accuracies), `BNS_BIAS`/`BNS_ACCBIAS` (IMU
bias/SF + accuracies), `BNS_ACC` (pos/vel/att accuracies), `GNSS_STS`/`GNSS_ACC`/`GNSS_STATS`
(GNSS status/DOP/counters), `GNSS_PGC`/`GNSS_SGC`/`GNSS_EGC` (primary/secondary/external GNSS
card diagnostics), `BNS_STS` (top-level nav/connection status), `BNS_STATS` (comms/IMU
counters), `BNS_GAD` (generic aiding), `AID_INNOV` (Kalman innovations), `BNS_WSP` (wheel
speed), `BNS_TRG`/`BNS_CAM` (trigger/camera events), `BNS_LOC` (local coordinate frame),
`META`/`TIME`/`BASE_STS`.

---

## 1. Core nav batch (position/velocity/attitude/IMU, ~100Hz)

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| Lat, Lon, Alt | Lat, Lon, Alt | BNS_SDN | confirmed | same units (deg, deg, m) |
| Vn, Ve, Vd | Vn, Ve, Vd | BNS_SDN | confirmed | m/s |
| Heading, Pitch, Roll | Heading, Pitch, Roll | BNS_SDN | confirmed | deg |
| Ax, Ay, Az | Ax, Ay, Az | BNS_SDN | confirmed (corrected) | Initial pass wrongly called this a frame mismatch. NCOM's own manual (Table 4) describes Ax/Ay/Az as "the host object's acceleration... after the IMU to host attitude matrix has been applied" — i.e. already vehicle frame (per `mobile.vat`'s IMU→vehicle rotation), same as UCOM's Ax/Ay/Az ("acceleration in the x axis of the vehicle frame"). Direct match, not a design decision. |
| Wx, Wy, Wz | Wx, Wy, Wz | BNS_SDN | confirmed (corrected) | Same correction as Ax/Ay/Az — NCOM's Wx/Wy/Wz are "the host object's angular rate... after the IMU to host attitude matrix has been applied" (vehicle frame), matching UCOM's Wx/Wy/Wz ("angular rate along the x axis of the vehicle frame"). Direct match. |
| NavStatus | InsNavMode | BNS_STS | confirmed | explicit rename in Renamed Signals table |
| GpsTime, GpsSeconds, GpsMinutes | Nano | TIME | NOT FOUND (structural) | UCOM has no minutes/seconds split — single `Nano` (S64, ns, GNSS-time epoch) on the TIME source. Would need re-deriving GpsWeek/GpsSeconds from Nano. |
| UtcTime | — | — | NOT FOUND (see decision below) | No UTC/leap-second signal anywhere in the manual (searched "utc"/"leap second" — zero hits) — UCOM exposes GNSS time (`Nano`) only. Ben's proposal: don't chase a device-side offset table (none exists); just use GNSS time directly and hold the fixed GNSS↔UTC leap-second offset (currently 18s, unchanged since Dec 2016) as a `config.yaml` value, updated by hand on the rare occasion IERS declares a new leap second. Remember GNSS time and NTP/system time are two different things — this offset is GNSS-to-UTC, not a clock-sync correction. |

## 2. GNSS mode / satellite-count fields

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| GpsPosMode | GnssPosMode | GNSS_STS | confirmed | renamed (Renamed Signals table) |
| GpsVelMode | GnssVelMode | GNSS_STS | confirmed | renamed |
| GpsAttMode | GnssAttMode | GNSS_STS | confirmed | renamed |
| GpsDiffAge | GnssDiffAge | GNSS_STS | confirmed | renamed (table: `DiffAge`→`GnssDiffAge`) |
| GpsNumObs | GnssPosNumSats | GNSS_STS | confirmed | renamed (table) |
| NumSatsUsedPos | GnssPosNumSatsUsed | GNSS_STS | confirmed | "Number of satellites used in position solution" |
| NumSatsUsedVel | GnssVelNumSatsUsed | GNSS_STS | confirmed | "...used in velocity solution" |
| NumSatsUsedAtt | GnssAttNumSats | GNSS_STS | confirmed | renamed (table: `HeaSatUsed`→`GnssAttNumSats`), also listed directly in GNSS_STS |
| NumGpsDiffL1, NumGpsDiffL2 | GnssDiffNumGpsL1, GnssDiffNumGpsL2 | GNSS_STS | likely | plausible match by purpose; not unit-checked (raw corrections count vs differential-observation count may differ semantically) |
| NumGloDiffL1, NumGloDiffL2 | GnssDiffNumGlonassL1, GnssDiffNumGlonassL2 | GNSS_STS | likely | same caveat as above |
| GpsPosReject | GnssPosReject | GNSS_STATS | confirmed | "Number of consecutive GNSS position updates rejected" |
| GpsVelReject | GnssVelReject | GNSS_STATS | confirmed | |
| GpsAttReject | GnssAttReject | GNSS_STATS | confirmed | |
| HDOP, PDOP, VDOP | HDOP, PDOP, VDOP | GNSS_ACC | confirmed | identical names/units |
| DGpsNtripStatus | GnssDiffNtripStatus | GNSS_STS | confirmed | "NTRIP state machine status" |
| DGpsChars, DGpsCharsSkipped, DGpsPkts | GnssDiffChars, GnssDiffCharsSkipped, GnssDiffPkts | GNSS_STATS | confirmed | direct counterparts, same purpose |
| BaseStationId | BaseStationID | BASE_STS | confirmed | case-only rename |
| BaseLineLength, BaseLineLengthAcc | BaseLineLength, BaseLineLengthAcc | GNSS_CFG | confirmed | identical names |
| HeadQuality, HeadSearchType/Status/Ready/Init/Num, HeadSearchMaster/Slave1/2/3/Time/Constr | — | — | **NOT FOUND — genuine open question, see below** | No dual-antenna heading-ambiguity-search *engine diagnostic* signals anywhere in the manual (searched "search"/"ambiguity" — zero hits, confirmed on a second pass). There IS a message ID 44 "Orientation aiding of the INS in dual-antenna frame" (`AttAidingStreamID`, `AttPitchDiff`, `AttHeadingDiff`) but that's an aiding-innovation update (how much a dual-antenna fix disagreed with the INS prediction), not the internal ambiguity-search-in-progress status these NCOM fields report. Ben notes OXTS has been rewriting the RTK engine and these ambiguity-search diagnostics may genuinely no longer apply to how the new engine works (rather than being an oversight) — this is the one item in the whole comparison that's a real "ask OXTS support" question rather than something resolvable from the manual alone. If the new RTK engine has its own equivalent search/fix-quality diagnostics (single-antenna position fix search, or dual-antenna orientation fix search), those aren't documented under any name we searched for. |
| GnssRawL1Enabled, GnssRawL2Enabled, GnssRawL5Enabled, GnssRawDopEnabled, GnssRawRngEnabled | — | — | NOT FOUND | receiver raw-output config toggles; no equivalent found |
| GnssGlonassEnabled, GnssGalileoEnabled | — | — | NOT FOUND | same — config toggle bitfield not found; likely superseded by the much richer per-constellation counters in GNSS_STS (GnssPosNum*, HeadNum*, etc.) which imply these constellations are simply always-on/reported rather than toggled |
| L1DiffEnabled, L2DiffEnabled, PsrDiffEnabled, SBASEnabled | — | — | NOT FOUND | augmentation-mode enable bits; not present |
| OmniVBSEnabled, OmniHpEnabled, OmniFreq, OmniSNR, OmniStatus, OmniSerial, OmniTrackTime | — | — | NOT FOUND | Omnistar-specific fields; OXTS appears to have dropped Omnistar-specific signals from UCOM entirely (deprecated satellite augmentation service) |

## 3. GNSS receiver diagnostics (primary/secondary/external cards)

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| GpsPrimaryNumSats | GnssInt1_NumSats | GNSS_PGC | confirmed | "Number of Satellites Tracked" |
| GpsPrimaryAntPower | GnssInt1_AntPower | GNSS_PGC | confirmed | |
| GpsPrimaryAntStatus | — | — | NOT FOUND | GNSS_PGC (internal card) has no AntStatus signal — only GNSS_EGC (external) does |
| GpsSecondaryNumSats | GnssInt2_NumSats | GNSS_SGC | confirmed | |
| GpsSecondaryAntPower | GnssInt2_AntPower | GNSS_SGC | confirmed | |
| GpsSecondaryAntStatus | — | — | NOT FOUND | same as primary — no per-internal-card AntStatus |
| GpsExternalNumSats | GnssExt1_NumSats | GNSS_EGC | confirmed | |
| GpsExternalAntPower | GnssExt1_AntPower | GNSS_EGC | confirmed | |
| GpsExternalAntStatus | GnssExt1_AntStatus | GNSS_EGC | confirmed | external card *does* have this one |
| GpsPrimaryPosMode, GpsSecondaryPosMode, GpsExternalPosMode | — | — | NOT FOUND | no per-card position-mode breakdown in GNSS_PGC/SGC/EGC; only the single unified GnssPosMode (GNSS_STS) exists |
| GpsPrimaryBaud, GpsSecondaryBaud, GpsExternalBaud | — | — | NOT FOUND | |
| GpsPrimaryCoreNoise/CoreTemp/CpuUsed/SupplyVolt (and Secondary/External equivalents) | — | — | NOT FOUND | receiver-board diagnostics (temperature, noise, CPU load, supply voltage) have no UCOM equivalent anywhere in the manual |
| GpsPrimarySetPosRate/SetVelRate/SetRawRate, GpsSecondarySetRawRate | — | — | NOT FOUND | receiver output-rate config registers; not present |
| GpsPrimary, GpsSecondary (hardware-type enum) | — | — | NOT FOUND | see §7 hardware-ID fields |

## 4. IMU bias / scale-factor + accuracies

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| WxBias, WyBias, WzBias | WxBias, WyBias, WzBias | BNS_BIAS | confirmed | deg/s, identical names |
| AxBias, AyBias, AzBias | AxBias, AyBias, AzBias | BNS_BIAS | confirmed | m/s^2 |
| WxSf, WySf, WzSf | WxSf, WySf, WzSf | BNS_BIAS | confirmed | |
| AxSf, AySf, AzSf | AxSf, AySf, AzSf | BNS_BIAS | confirmed | |
| WxBiasAcc, WyBiasAcc, WzBiasAcc | WxBiasAcc, WyBiasAcc, WzBiasAcc | BNS_ACCBIAS | confirmed | |
| AxBiasAcc, AyBiasAcc, AzBiasAcc | AxBiasAcc, AyBiasAcc, AzBiasAcc | BNS_ACCBIAS | confirmed | |
| WxSfAcc, WySfAcc, WzSfAcc | WxSfAcc, WySfAcc, WzSfAcc | BNS_ACCBIAS | confirmed | |
| AxSfAcc, AySfAcc, AzSfAcc | AxSfAcc, AySfAcc, AzSfAcc | BNS_ACCBIAS | confirmed | |

## 5. Position/velocity/orientation accuracies

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| NorthAcc, EastAcc | NorthAcc, EastAcc | BNS_ACC | confirmed | |
| AltAcc | AltAcc | BNS_ACC | confirmed | renamed from `DownAcc` per table — but our own field is already spelled `AltAcc` in current code, so this is a direct name match already (NCOM's raw name is `DownAcc`, we already use `AltAcc` as our dict key) |
| VnAcc, VeAcc, VdAcc | VnAcc, VeAcc, VdAcc | BNS_ACC | confirmed | |
| HeadingAcc, PitchAcc, RollAcc | HeadingAcc, PitchAcc, RollAcc | BNS_ACC | confirmed | |

## 6. Lever arms

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| GAPx, GAPy, GAPz | GAPx, GAPy, GAPz | BNS_OPT | confirmed | "Distance to Primary GNSS Antenna" |
| GAPxAcc, GAPyAcc, GAPzAcc | GAPxAcc, GAPyAcc, GAPzAcc | BNS_ACCOPT | confirmed | |
| ZeroVelLeverArmX/Y/Z | ZeroVelLeverArmX/Y/Z | INS_CFG | confirmed | identical names |
| ZeroVelLeverArmX/Y/ZAcc | ZeroVelLeverArmX/Y/ZAcc | INS_CFG | confirmed | |
| NoSlipLeverArmX/Y/Z | NoSlipLeverArmX/Y/Z | INS_CFG | confirmed | UCOM describes this as "Vertical Advanced Slip Point" — same signal name, just re-described |
| NoSlipLeverArmX/Y/ZAcc | NoSlipLeverArmX/Y/ZAcc | INS_CFG | confirmed | |
| WSpeedLeverArmX/Y/Z | WSpeedLeverArmX/Y/Z | BNS_WSP | confirmed | |
| WSpeedLeverArmX/Y/ZAcc | WSpeedLeverArmX/Y/ZAcc | BNS_WSP | confirmed | |
| RemoveLeverArmX/Y/Z | RemoteLeverArmX/Y/Z | INS_CFG | confirmed | NCOM's own field name looks like a typo ("Remove" vs "Remote") — UCOM's "RemoteLeverArmX" ("Output displacement lever-arm X") is clearly the same concept, correctly spelled |

## 7. Antenna orientation, misalignment, vehicle-frame rotations

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| AtH, AtP | AtH, AtP | BNS_OPT | confirmed | "Heading/Pitch Orientation of the GNSS Antenna" |
| AtHAcc, AtPAcc | AtHAcc, AtPAcc | BNS_ACCOPT | confirmed | |
| HeadingMisAlign, HeadingMisAlignAcc | HeadingMisAlign, HeadingMisAlignAcc | BNS_OPT / BNS_ACCOPT | confirmed | |
| VehHeading, VehPitch, VehRoll | Imu2VehHeading, Imu2VehPitch, Imu2VehRoll | BNS_CFG | likely | UCOM's description "Heading/Pitch/Roll of the vehicle in the RT co-ordinate frame" matches NCOM's "RT to vehicle rotation" comment; not independently unit/sign-verified |
| Ned2SurfHeading/Pitch/Roll | Ned2SurfHeading/Pitch/Roll | BNS_CFG | confirmed | identical names |
| OpHeading, OpPitch, OpRoll | Veh2OutHeading/Pitch/Roll (?) | BNS_CFG | likely | NCOM comment says "vehicle to output rotation, very rarely used" which matches BNS_CFG's `Veh2Out*` description almost exactly; not unit-verified. (Note BNS_CFG *also* has `Surf2Out*` — a plausible second candidate; genuinely ambiguous without vendor confirmation.) |

## 8. Slip points (additional slip points 1–8)

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| SlipPoint1X..8Z (24 fields) | MeasPt1_PointXv/PointYv/PointZv .. MeasPt8_PointXv/PointYv/PointZv | INS_CFG | confirmed | UCOM's own description text is verbatim "Distance to the Additional Slip Point in X/Y/Z direction" for MeasPt1..8 — matches NCOM's "Additional slip point N" comments exactly. Renamed from `SlipPointN{X,Y,Z}` to `MeasPtN_Point{X,Y,Z}v`. |

## 9. Reference/local coordinate frame

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| RefFrameLat, RefFrameLon, RefFrameAlt, RefFrameHeading | RefLat, RefLon, RefAlt, RefHeading | BNS_LOC | confirmed | "Local Co-ordinates Origin Latitude/Longitude/Altitude/Heading" — renamed, dropped "Frame" |
| RefLatRadius, RefLonRadius, RefHeadingCos, RefHeadingSin | — | — | **superseded, not "not found"** | These are *not* raw NCOM fields — they're values our own `computeRefFrame()` derives locally from RefFrameLat/Lon/Alt/Heading to convert LLA↔local NED. UCOM's BNS_LOC source outputs the local-frame result directly (`RefFrameX`, `RefFrameY`, `RefFrameVelX`, `RefFrameVelY`, `RefFrameTrack`, `RefFrameYaw`, `RefFrameNorthing`, `RefFrameEasting`) computed on-device, so this whole local-coordinate-math helper becomes unnecessary rather than needing a mapped replacement. |

## 10. Wheel speed

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| WSpeedCount | WSpeedCount | BNS_WSP | confirmed | "Cyclic wheelspeed input counts" |
| WSpeedScale, WSpeedScaleStd | WSpeedScale, WSpeedScaleStd | BNS_WSP | confirmed | |
| WSpeedTimeUnchanged | WSpeedTimeUnchanged | BNS_WSP | confirmed | |
| WSpeedTime | WSpeedNano | BNS_WSP | likely | renamed + unit change: NCOM's WSpeedTime is a derived GpsTime-based datetime, UCOM's WSpeedNano is a raw ns timestamp ("Wheel speed time of last change") — same underlying purpose |
| OptionWSpeedDelay, OptionWSpeedZVDelay, OptionWSpeedNoiseStd | OptionWSpeedDelay, OptionWSpeedZVDelay, OptionWSpeedNoiseStd | BNS_WSP | confirmed | identical names |

## 11. GAD (Generic Aiding Data) — see dedicated section at end for the stream-ID question

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| GadStreamId | GADLatestStreamID | BNS_GAD | confirmed | "Stream ID of the most recently received GAD packet" |
| GadReject | GADLatestStatus | BNS_GAD | likely | closest match by position/purpose; NCOM's GadReject is a reject-code byte, UCOM's GADLatestStatus is "Status of the latest GAD packet received" — plausibly the same but not unit/enum-verified. There's also GADPosStatus/GADVelStatus/GADAttStatus/GADAngRateStatus (per aiding-type reject status), which may be a richer replacement. |
| GadTime | — | — | likely/NOT FOUND | no direct "GadTime" signal; BNS_GAD has no per-stream timestamp signal (GADNumEarly/GADNumLate/GADNumScheduled are *counts*, not a time value) |
| GadInn1, GadInn2, GadInn3 | — | — | NOT FOUND | AID_INNOV's innovation signals (InnPosX/Y/Z, InnVelX/Y/Z, InnHeading, InnPitch, etc.) are all named by *physical quantity*, not by generic "GAD innovation slot 1/2/3". If a GAD stream supplies e.g. a position update, its innovation would appear as InnPosX/Y/Z etc. — but there is no generic GadInn1/2/3 passthrough signal that mirrors NCOM's raw byte-level innovations regardless of what they represent. |

## 12. Trigger / camera digital I/O events

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| Trig1FallingCount, Trig1RisingCount | Trig1FallUpdateCount, Trig1RiseUpdateCount | BNS_TRG | confirmed | renamed (Count → UpdateCount) |
| Trig2FallingCount, Trig2RisingCount | Trig2FallUpdateCount, Trig2RiseUpdateCount | BNS_TRG | confirmed | |
| Trig1FallingTime, Trig1RisingTime, Trig2FallingTime, Trig2RisingTime | — | — | NOT FOUND (structurally superseded) | BNS_TRG's Natural Output Type is `OnTrigger` — the message itself is emitted at the trigger instant and carries its own Nano timestamp, so an explicit "trigger time" signal isn't needed; the event time comes from the message envelope, not a payload signal. |
| Digital1OutCount, Digital2OutCount | DigitalOutUpdateCount, DigitalOut2UpdateCount | BNS_CAM | confirmed | renamed |
| Digital1OutTime, Digital2OutTime | — | — | NOT FOUND (structurally superseded) | same reasoning as trigger times — BNS_CAM is also `OnTrigger`, event time is the message timestamp |

## 13. Hardware identification / firmware / misc board info

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| SerialNumber | SerialNumber | META | confirmed | |
| DevId | DevID | META | confirmed | case rename |
| DiskSpace, FileSize | DiskSpace, FileSize | META | confirmed | |
| SupplyVoltage | SupplyVolt | META | confirmed | renamed (dropped "age") |
| NComEncoderVersion | UCOMVersion | META | likely | "The UCOM version of this UCOM output" — analogous purpose (encoder/protocol version), not confirmed identical semantics |
| CpuPcbType, FrontPcbType, InterPcbType, InterSwId, HwConfig, ImuType, GpsPrimary, GpsSecondary (hw-type enums) | — | — | NOT FOUND | no hardware-identification signals of this kind found anywhere in the Sources section |
| GpsSetType, GpsSetFormat, DualPortRamStatus | — | — | NOT FOUND | Trimble-era receiver config registers; absent |
| FirmwareExpiryDate | — | — | NOT FOUND | |
| OutputLatency | — | — | NOT FOUND | no explicit output-latency signal; each message's own Nano timestamp may serve a similar diagnostic role indirectly |
| ImuRate | ImuRate | INS_CFG | confirmed | "Maximum output rate of the IMU" |
| ImuLoopTime, OpLoopTime, ImuTimeDiff, ImuTimeMargin, BnsLag | — | — | NOT FOUND | internal real-time-processing timing diagnostics; not present |
| TimeMismatch | TimeMismatch | BNS_STATS | confirmed | identical name |
| WifiConnectionStatus | WiFiConnectionStatus | BNS_STS | confirmed | rename (WiFi casing) — UCOM also adds WiFiAccessPointStatus, PTPStatus (new) |
| UmacStatus | UmacStatus | BNS_STS | confirmed | identical |
| BlendedMethod | Generator | BNS_CFG | likely | "Blending processing method" description matches, not confirmed as literally the same enum |
| Undulation | Undulation | BNS_SDN | confirmed | |
| DatumEllipsoid | DatumEllipsoid | BNS_CFG | confirmed | |
| DatumEarthFrame | DatumFrame | BNS_CFG | confirmed | renamed, dropped "Earth" |
| OsVersion1/2/3, OsScriptId | — | — | NOT FOUND | no OS/script version signals found |

## 14. Option / configuration parameters

| NCOM field | UCOM signal | Source | Confidence | Notes |
|---|---|---|---|---|
| OptionLevel | OptionLevel | BNS_CFG | confirmed | |
| OptionVibration | OptionVibration | BNS_CFG | confirmed | |
| OptionHeading | OptionHeading | BNS_CFG | confirmed | |
| OptionInitSpeed | OptionInitSpeed | BNS_CFG | confirmed | |
| OptionTopSpeed | OptionTopSpeed | BNS_CFG | confirmed | |
| OptionHeave | OptionHeave | BNS_CFG | confirmed | |
| OptionSZVDelay, OptionSZVPeriod | OptionSZVDelay, OptionSZVPeriod | BNS_CFG | confirmed | "Garage mode zero velocity delay/period" |
| OptionNSDelay, OptionNSPeriod, OptionNSAngleStd, OptionNSHAccel, OptionNSVAccel, OptionNSSpeed, OptionNSRadius | (same names) | BNS_CFG | confirmed | all 7 identical names, "Lateral advanced slip ..." |
| OptionHLDelay, OptionHLPeriod, OptionHLAngleStd | (same names) | BNS_CFG | confirmed | "Heading lock ..." |
| OptionStatDelay, OptionStatSpeed | OptionStatDelay, OptionStatSpeed | BNS_CFG | confirmed | |
| OptionStartDelay | — | — | NOT FOUND | Note: our own decoder has a latent bug here — `decodeStatus42` sets `self.status['OptionStatDelay']` but then checks/deletes `self.status['OptionStartDelay']` (a key that was never set), so `OptionStartDelay` never actually appears with a value in practice. Independent of that bug, no UCOM equivalent was found either. |
| OptionGpsAcc | — | — | NOT FOUND | |
| OptionUpd | OptionUdp | INS_CFG | likely | "Configuration of Ethernet UDP1 output" — plausible rename/typo-correction of NCOM's "OptionUpd", same byte position among serial/UDP config options, not independently confirmed |
| OptionsSer1, OptionsSer2, OptionSer3 | OptionSer1, OptionSer2, OptionSer3 | INS_CFG | confirmed | "Configuration of Serial N output (NCOM/NMEA, etc)" — ours plural ("OptionsSerN"), UCOM singular ("OptionSerN") |
| OptionSer1Baud, OptionSer2Baud, OptionSer3Baud | OptionSer1Baud, OptionSer2Baud, OptionSer3Baud | INS_CFG | confirmed | |
| OptionCanBaud | OptionCanBaud | INS_CFG | confirmed | |

---

## GAD / status-95 per-stream-ID question — findings

**UCOM does not natively separate GAD status by stream ID either.** The `BNS_GAD` source
(Table 93, "Generic aiding") is a single flat message whose signals are explicitly described
as covering only "the most recently received GAD packet" / "the latest GAD stream":

- `GADLatestStreamID` — "Stream ID of the **most recently received** GAD packet"
- `GADLatestStatus` — "Status of the **latest** GAD packet received"
- `GADNumEarly` / `GADNumLate` / `GADNumScheduled` — counts "on the **latest** GAD stream"
- `GADPosStatus`, `GADVelStatus`, `GADAttStatus`, `GADAngRateStatus` — per *aiding-type*
  (position/velocity/attitude/angular-rate) status, not per stream ID

Similarly, `AID_INNOV` (Innovations source) exposes `PosAidingStreamID`, `VelAidingStreamID`,
`AttAidingStreamID` — "ID of the stream providing position/velocity/attitude aiding to the
INS" — again singular, describing only whichever stream most recently supplied that type of
update, with no array/indexed structure and no message-per-stream-ID design.

So **UCOM's data model has the exact same structural limitation as our current NCOM
decoder**: a new GAD packet's status simply overwrites the previous one's fields in the same
message, regardless of stream ID. There is no protocol-level mechanism (indexed signals, one
message per stream, a repeating/array signal type, etc.) that would let a UCOM consumer
recover concurrent per-stream-ID GAD status "for free."

**Implication for the requested per-stream separation:** achieving "report each GAD stream ID
separately, don't clobber one stream's status with another's" will require the *same*
application-level bookkeeping under UCOM as it would under NCOM — i.e., our own decoder
keeping a `{stream_id: {reject, status, time, ...}}` dict and updating only the entry named by
`GADLatestStreamID`/`PosAidingStreamID` etc. each time the message changes. This is not a
protocol upgrade UCOM gives us — it's still work we'd have to build ourselves, just as our
current NCOM code's own comment already anticipated ("might be better to have an array, one
for each GAD stream ID").

One partial improvement: UCOM breaks GAD status out by *aiding type* (`GADPosStatus`,
`GADVelStatus`, `GADAttStatus`, `GADAngRateStatus`) where NCOM only had one generic
`GadReject` byte — so if two concurrent GAD streams feed different aiding types (e.g. one
position-only stream and one attitude-only stream), UCOM can already distinguish those without
extra bookkeeping. It's only when two streams compete for the *same* aiding type that the
stream-ID clobbering problem remains identical to today.

**Output timing confirms the per-stream-table approach is workable:** `BNS_GAD`'s "Natural
Output Type" is `OnChange` — per the manual's Output Types section, an OnChange message is
only emitted "when there is fresh data in at least one of the signals contained within the
message." In practice that means one `BNS_GAD` message per GAD update received (not a fixed
poll rate), which is exactly the "one status output each time a GAD message is received"
behaviour Ben described. So the plan of building our own `{stream_id: last_status}` table —
updating just the entry named by `GADLatestStreamID` each time a `BNS_GAD` message arrives —
is sound and gives aruco's planned position/heading-stream status page (accept/reject,
innovations, maybe a scrolling chart) what it needs. Note this bookkeeping is independent of
the NCOM→UCOM migration itself — nothing stops building it against the current NCOM decoder
today.

---

## Summary counts

- Total NCOM fields evaluated: 304
- Confirmed UCOM equivalent (same or renamed, verified): ~187 (Ax/Ay/Az and Wx/Wy/Wz corrected
  from an initial false "frame mismatch" — see §1 note — both are direct vehicle-frame matches)
- Likely UCOM equivalent (plausible, not fully unit/enum-verified): ~20
- NOT FOUND (no UCOM equivalent located): ~93, concentrated in:
  - GNSS receiver board diagnostics (baud, core temp/noise, CPU load, supply volt, per-card pos mode, set-rate registers) — primary/secondary/external, ~30 fields
  - Dual-antenna heading-ambiguity search (HeadQuality/HeadSearch*) — 11 fields — **the one genuine open question in this whole report; recommend asking OXTS support directly whether the rewritten RTK engine has a replacement diagnostic, since it isn't documented under any name we could find**
  - Omnistar-specific status — 7 fields — confirmed fine to drop, deprecated service, gone for good
  - Hardware identification (PCB types, InterSwId, HwConfig, ImuType, GpsPrimary/Secondary hw enum, GpsSetType/Format, DualPortRamStatus) — ~11 fields
  - Real-time processing timing diagnostics (ImuLoopTime, OpLoopTime, ImuTimeDiff/Margin, BnsLag, OutputLatency, NComEncoderVersion-adjacent) — ~6 fields
  - Explicit trigger/digital-output *times* (superseded by message-level timestamps, structurally not "missing") — 6 fields
  - Firmware/OS version and misc (FirmwareExpiryDate, OsVersion1-3, OsScriptId, TimeUtcOffset) — 6 fields
  - GNSS raw-signal enable toggles and augmentation enable bits — ~11 fields
  - GAD innovation passthrough (GadInn1/2/3) and GadTime — 4 fields

## Addendum: fields set via `_updateLE16`/`_updateLE32`/`_updateLE8`/`_updateInnovation`

The 304-field list handed to the original comparison only caught fields assigned via a literal
`self.status['Name'] = ...`/`self.nav['Name'] = ...` pattern — it missed everything set through
`ncomrx.py`'s helper functions, which take the field name as a string argument instead (e.g.
`self._updateLE16(..., 'GpsPrimaryChars')`, `self._updateInnovation('InnPosX', ...)`). That's
roughly another 30 fields: the raw (non-`Filt`) innovation values (`InnPosX/Y/Z`, `InnVelX/Y/Z`,
`InnPitch`, `InnHeading`, `InnZeroVelX/Y/Z`, `InnNoSlipH`, `InnHeadingH`, `InnWSpeed` — the
`Filt` suffix versions shown on the status page are a locally-computed smoothed value, not a raw
signal, so they don't need a UCOM counterpart at all), and per-link decoder health counters
(`GpsPrimaryChars/Pkts/CharsSkipped/OldPkts`, `GpsSecondaryChars/Pkts/CharsSkipped/OldPkts`,
`GpsExternalChars/Pkts/CharsSkipped/OldPkts`, `ImuChars/Pkts/ImuSkipped/ImuMissedPkts`,
`CmdChars/Pkts/CharsSkipped/Errors`).

Checked all of these directly against the manual: **every one has a UCOM equivalent.** The raw
innovations are all present by name on `AID_INNOV` (confirmed: `InnHeadingH`, `InnZeroVelX/Y/Z`,
`InnNoSlipH`, `InnWSpeed` all found verbatim). The decoder health counters follow the exact same
`GnssInt1_`/`GnssInt2_`/`GnssExt1_` prefix convention already documented in §3 for
primary/secondary/external GNSS card diagnostics (e.g. `GpsPrimaryChars`→`GnssInt1_Chars`,
`GpsPrimaryCharsSkipped`→`GnssInt1_CharsSkipped`), on `GNSS_STATS`; the IMU/Cmd counters are on
`BNS_STATS` under their existing names. No new gaps found — the summary counts above are
unaffected.

---

## Configuring UCOM output by hand (`mobile.dbu`) — findings

Checked against the manual plus a live pull of Amundsen's `oxts.dbu`/`oxts.dbs` (fetched via FTP,
the same read-only mechanism `oxts-nav`'s `download_xnav_config()` already uses — no `mobile.dbu`
exists on this xNAV650 yet, confirming UCOM output has never been enabled/configured on this
device at all; `oxts.dbu`'s 93-message catalog is what's possible, not what's active).

**What's actually editable per message** (confirmed against all 93 catalog entries, not just the
manual's one worked example): `MessageEnabled` (bool), `OutputFrequencyDecimation` (int, relative
to `FrequencyOutputType`'s natural rate), `FrequencyOutputType` (`"IMU"` = 100Hz base, `"GNSSRaw"`
= the GNSS receiver's native rate, or absent for `OnChange`-only messages), `OutputType`
(`"FrequencyBased"` or `"OnChange"` — every one of the 93 default messages uses one of these two;
trigger-based output is a separate mechanism layered via the `PossibleTriggerTypes` enum in the
header, not represented as a third `OutputType` string in this catalog), `MessageTiming` (`"SDN"` for
every message here — `"GNSS"` is the other documented option).

**Message ID 100 ("SDN time offsets") is not a bandwidth-pacing tool** — despite the name, it
reconciles the SDN clock domain against GNSS/PTP time, unrelated to scheduling.

**No phase/stagger mechanism was found anywhere** — not in the manual, not in any of the 93 live
catalog entries. This matters more for UCOM than it did for NCOM: NCOM bundles the 100Hz nav
batch plus exactly one rotating status channel into a single fixed-format packet, so staggering was
free/automatic. UCOM instead sends **one full UDP datagram per message per output tick** (per the
manual's "UCOM Payload Key Components" section — "the payload consists of a UCOM message,"
singular). So several messages configured at the same decimation (e.g. multiple diagnostics all set
to 1Hz) will most likely all fire on the same underlying tick, back-to-back, with nothing documented
to spread them out. This is a genuine open question — can't be resolved from the manual, worth
either (a) asking OXTS directly (could ride along with the RTK-search question), or (b) testing
empirically once something is actually configured, by watching real UDP arrival timestamps.

**Hard limits to design the message set around:** a single message is capped at 1452 bytes (MTU-
driven, to avoid IPv4 fragmentation — hard constraint), and OXTS's own recommendation is to keep
the *whole* configured set under ~600 bytes / ~10 messages, with 100Hz as the max rate for any one
message. Message IDs 64512–65535 are reserved for our own custom messages — worth considering
bundling several wanted signals into one or two custom messages (rather than enabling many small
official per-topic ones separately) as a way to control total packet count, since custom messages can
draw signals from any source but currently can't remap their native type/scale/offset (output as-is).

---

## Implemented: `ucomrx.py` + `mobile.dbu` (2026-07-23)

Built `oxts-nav/ucomrx.py` — a UCOM decoder, same `nav`/`status`/`connection` dict shape as
`ncomrx.py`, one explicit `decodeMessageN` (or `decodeMessageNvV`) method per message defined in
`oxts-nav/documentation/oxts.dbu` (93 total, mechanically generated from that file rather than
hand-transcribed, then spot-checked against the manual and a decode round-trip test — see the
file's own docstring for the reasoning on why this is hardcoded/explicit rather than a generic
schema-interpreting decoder). Two facts used by the decoder that aren't obvious from the manual,
confirmed against OXTS's own reference decoder instead (`github.com/OxfordTechnicalSolutions/
ucom-decoder`, the C++ core, not the Python bindings):

1. Signal `ScaleFactor`/`Offset` are never applied when decoding — values are used as-is in their
   native unit (confirmed: the reference's `ucom_data.cpp` never reads those fields).
2. The CRC is **not** `zlib.crc32()` — OXTS's variant seeds at 0 (not `0xFFFFFFFF`) and only
   complements the result once, at the end. Using the standard zlib CRC would silently reject
   every real packet.

`oxts-nav/documentation/` holds the actual `oxts.dbu`/`oxts.dbs` pulled from Amundsen's xNAV650 via
FTP (2026-07-23 — confirming no `mobile.dbu` existed on the device before this). The generated
`mobile.dbu` itself lives at `xnav-config/mobile.dbu.txt` (auto-downloaded alongside the NCOM config
files, not documentation — see `oxts-nav-prd.md`), enabling exactly what's needed right now:
Heartbeat (0) and SDN time offsets (100,
both default-enabled, left alone), nav frame PVA (1v1, 100Hz — the core position/velocity/heading
loop), PVA accuracies (3v1, `OnChange`), GNSS Status (5v1, 10Hz), and the three GAD-visibility
messages that motivated this whole exercise: GAD statuses (37 — note its *message* defaults to
`FrequencyBased`/100Hz even though its underlying *source* is naturally `OnChange`, corrected here
to actually be `OnChange` to match 38/40 and avoid needlessly hammering the link), position aiding
innovations (38), attitude aiding innovations (40). Total ~233 payload bytes across 8 messages —
comfortably under OXTS's ~600-byte/~10-message guidance. Velocity aiding innovations (39) and INS
Status (54, redundant with `InsNavMode` already in message 5) deliberately left disabled.

**Concrete finding for the aruco GAD-visibility use case**: position aiding (38) and attitude aiding
(40) already have independent `PosAidingStreamID`/`AttAidingStreamID` fields — so if aruco's marker
position stream and marker heading stream use different aiding types (position vs. attitude), as
they do, per-stream separation may already come for free, with no extra bookkeeping needed. The
earlier-flagged clobbering problem only remains if two streams ever compete for the *same* aiding
type.

**Not done yet / explicitly deferred:**
- **Not wired into `app.py`/`ncomrx_thread.py`/`nav_feed.py`** — this is a standalone decoder only,
  matching the earlier-agreed plan (build and validate independently before any cutover).
- **`mobile.dbu` has not been installed on Amundsen** — it's generated and sitting in the repo;
  installing it means FTP-uploading it to the device, which actually changes live device behaviour
  (turns UCOM output on) and hasn't been done, on purpose, without explicit sign-off first.
- **SDN→GNSS time reconciliation is unresolved.** Every message header carries only an "arbitrary"
  SDN-clock timestamp (nanoseconds since power-on); message 100 gives the offset to real GNSS time,
  but `ucomrx.py` doesn't yet apply it anywhere (no `GpsTime`-equivalent is computed). Needed before
  this can replace NCOM's `GpsTime`/`UtcTime` for real.
- ~~The NCOM→UCOM field rename... hasn't happened yet~~ **Done (2026-07-23).** `ncomrx.py`'s dict
  keys (84 fields — `GpsPosMode`→`GnssPosMode`, `NavStatus`→`InsNavMode`, etc., per the "confirmed"/
  "likely" rows in this document) now match `ucomrx.py`'s native UCOM names, plus every consumer
  that referenced the old names: `oxts-nav/templates/pages/{status,home}.html`,
  `navigate/templates/pages/run.html`, `aruco/templates/pages/add-marker.html`,
  `shared/web/static/ncom-strings.js` (also dropped its now-dead `HeadQuality` entry, following the
  earlier removal of the dual-antenna-search status rows). Deliberately NOT renamed: `OpHeading`/
  `OpPitch`/`OpRoll` — the mapping doc's own §7 flags a genuine ambiguity between UCOM's
  `Veh2Out*`/`Surf2Out*` as the real match, not resolved here, so the old name stays until that's
  settled rather than guessing. `LLA2NED`/`NED2LLA`'s `RefFrameLat`/`Lon`/`Alt` parameter names were
  also left alone — dead code, unused anywhere in the repo, not worth touching in this pass. Full
  test suite (145 tests across navigate/drive/aruco/shared) still passes; no test referenced any of
  the renamed names directly.

**Config switch (2026-07-23):** `oxts-nav/ucomrx_thread.py` added — mirrors `ncomrx_thread.py`'s
structure deliberately (own socket bound to UCOM's fixed port 50487, own per-IP decoder dict, own
lock), rather than a shared generic base class between the two, consistent with `ucomrx.py`'s own
"explicit over generic" design. A new `oxts-nav.protocol` config key (`ncom` or `ucom`, default
`ncom`) picks which thread `app.py` constructs at startup — restart-only, no dynamic switch, same
as every other config value in this project. Not yet tested against a live UCOM stream (nothing on
Amundsen is emitting UCOM yet — see "Not done yet" below); confirmed `ucomrx_thread` imports and
constructs cleanly in isolation. `nrxs.moreCalcs` (an unused extensibility hook already dead in
`ncomrx_thread.py` — nothing in the repo populates it) was deliberately not replicated in
`ucomrx_thread.py`.

**Fixed (2026-07-23): false "repeated UDP" storm on the real switch-over.** First live test after
flipping `protocol: ucom` showed `connection['repeatedUdp']` climbing ~200/s while `numPackets`
stayed frozen. Root cause (confirmed via `tcpdump` + a live capture, not guessed): `ucomrx_thread.py`
copied `ncomrx_thread.py`'s duplicate-UDP-delivery guard verbatim, which fingerprints each packet
with `binascii.crc32()` rather than comparing raw bytes. That's safe for NCOM (Batch A's real sensor
noise gives every packet enough entropy) but not for UCOM: several messages (Heartbeat in
particular) are almost entirely constant except one steadily-incrementing timer field, and CRC32 is
linear over GF(2) — a steady increment can land back on the exact same 32-bit CRC at a predictable
interval, causing real, reproducible false-duplicate hits, not a rare collision. Verified directly:
two genuinely different 24-byte Heartbeat packets (differing only in the arbitrary-time field and
the trailing UCOM CRC that depends on it) produced identical `binascii.crc32()` output. Fixed by
comparing raw packet bytes directly instead of a CRC fingerprint — packets here are small enough
(well under 100 bytes) that keeping the last 200 costs nothing. `ncomrx_thread.py` was deliberately
left alone — same theoretical exposure exists there too, but it's never manifested in years of real
NCOM use, and this pass was about fixing what's actually broken, not preemptively rewriting trusted
code.

Live-verified after the fix: `numPackets` climbing at a healthy ~200/s, `repeatedUdp` staying at 0,
`nav` populated with real position/heading, `status` populated with accuracies (`NorthAcc` etc.),
GAD position-aiding innovations (`PosAidingStreamID`, `InnPosX/Y/Z`), and the SDN time offset
(`SDNOFF`/`SDNOFFSource`).

**`connection['timeOffset']` implemented (2026-07-23) — resolves the open item above.** Empirically
confirmed against Amundsen's real xNAV650 (not documented in the manual): `GpsSecondsSinceEpoch =
headerArbitraryTimeNs/1e9 + SDNOFF_ns/1e9`, using the *same* GPS epoch (1980-01-06) `ncomrx.py`
already uses — computed time matched the system clock to the second. `UcomRx.decode()` now accepts
`machineTime` (mirroring `NcomRx.decode()`'s signature) and maintains `connection['timeOffset']` on
every packet once a GNSS-sourced `SDNOFF` (message 100, source enum `1`) has been seen, plus sets
`self.nav['GpsTime']` whenever nav (message 1) arrives — both live-verified. This means
`machine_time_to_gps()` (used directly by `aruco/app.py` for GAD timestamp correlation) now works
unchanged under either protocol. `ucomrx_thread.py` was renamed `nrx`→`urx` per Ben's request; this
broke two consumers that hardcoded `.nrx` assuming only `NcomRxThread` would ever be used —
`nav_feed.py` and `app.py`'s `/ws/nav` route (the one powering the whole live status page) — fixed
by having `app.py` pick the right attribute once, at thread-construction time, and pass it down
rather than either consumer guessing.

**`InsNavMode` was never being sent at all, not just misrouted.** Message 5 v1 (chosen for
`mobile.dbu` to save 9 bytes) dropped `InsNavMode` — only v0 has it, and message 54 ("INS Status",
the other carrier) was left disabled on the assumption v1 covered it. Fixed: `decodeMessage54` now
mirrors `InsNavMode` into `self.nav` too (matching `ncomrx.py`'s own dual-write of `NavStatus`, which
`status.html`'s "Navigation status" row depends on), and message 54 is now enabled in `mobile.dbu`
(10Hz, uploaded to Amundsen). Needs another xNAV650 reset to actually start being sent — GpsTime
above didn't need one (pure software), this one does.

**`mobile.dbu` replaced with a NAVconfig-generated one (2026-07-23).** Ben's hand-editing our
JSON turned out to be masking a real constraint: NAVconfig won't let you change the decimation on
some messages (5, 54 are fixed at 100Hz) — our earlier hand-crafted file forced them to 10Hz
anyway, which may have been an invalid configuration the firmware silently mishandled (a plausible
explanation for some of the earlier degraded-rate weirdness). Now trusting the GUI-validated file
over hand JSON edits. NAVconfig's set: our original 9 (0, 1v1, 3v1, 5v1, 37, 38, 40, 54, 100) plus
three more from the GAD family (36 GAD decoder information, 39 velocity aiding innovations, 41 GAD
packet statistics) — harmless extras, zero code cost since every message already has a decoder.

**UTC time implemented.** `GPS_UTC_OFFSET_S = -18` (fixed constant, GPS has been exactly 18s ahead
of UTC since the last leap second in Dec 2016 — see the constant's comment for where to check if
that ever changes) added alongside `GpsTime`'s computation in `decode()`.

**Innovation "Filt" fields implemented.** `_updateInnovationFilt()` replicates `ncomrx.py`'s
`_updateInnovation` fast-attack/slow-decay envelope (same 0.9/0.1 constants) for UCOM's raw
`InnPosX/Y/Z` (message 38) and `InnPitch`/`InnHeading` (message 40).

**Systematic status-page audit (2026-07-23):** wrote a one-off script cross-referencing every
`nav-`/`status-` field `status.html` binds against what `ucomrx.py` can produce and what's enabled.
Result: everything from messages 0/1/3/5/37/38/40/54/100 is fully accounted for. Everything else
still missing (`BaseStationID`, `GnssPosReject`/`VelReject`/`AttReject`, `BaseLineLength`,
`HeadingMisAlign`, `GnssPosNumSatsUsed`/`VelNumSatsUsed`, `TimeMismatch`, `UpTime`, the
`GnssInt1_`/`GnssInt2_`/`Imu` decoder-health counters, IMU bias/scale-factor, `DevID`/`DiskSpace`/
`FileSize`) is **not just disabled — none of these signals are packaged into any of OXTS's 93
predefined messages at all** (checked directly against `oxts.dbu`). Getting any of them requires
defining our own custom message (ID range 64512–65535, per the manual's "Custom Messages" section)
— a real, separate piece of design work, not a config flag flip.

**Custom messages confirmed working (2026-07-23).** Ben suspected NAVconfig might not actually
support creating them, separate from whether the firmware does — worth checking directly before
reporting anything to OXTS as broken. Tested with a minimal one-signal custom message (ID 64512,
just `UpTime`): showed up on the wire and decoded correctly. Conclusion for the OXTS report: the
*firmware* handles custom messages fine — this is a NAVconfig GUI gap, not a protocol/firmware
limitation, so "everything from NCOM is implemented" is basically true at the protocol level even
though it's not reachable through the normal tool.

Built a proper one from that: message **64513** ("GR6 custom status fields", 30 signals, 158-byte
payload, 1Hz), pulling in everything found missing from the status-page audit above except the two
`STR`-type fields (`BaseStationID`, `DevID` — deliberately left out, variable-length string parsing
inside a multi-signal message is real extra complexity for low payoff; can add later if wanted) —
`UpTime`, the three GNSS reject counters, `BaseLineLength`, `HeadingMisAlign`, both "used" sat
counts, `TimeMismatch`, the `GnssInt1_`/`GnssInt2_` decoder-health counters (note: these live on
sources `GNSS_PGC`/`GNSS_SGC`, not `GNSS_STATS` as originally guessed in the NCOM→UCOM rename —
worth double-checking that assumption if it's ever revisited), `ImuChars`/`ImuPkts` (`ImuSkipped`
has no UCOM equivalent anywhere — genuinely absent from `oxts.dbs`, not just unpackaged), IMU
bias/scale-factor (`WxBias` etc.), and `DiskSpace`/`FileSize`. `decodeMessage64513` written by hand
(not generated, since it's not part of the mechanical 93-message extraction) following the exact
same explicit style. Uploaded, service restarted, awaiting an xNAV650 reset to confirm live. Also
added a temporary test message (64514, `InsNavMode` alone) to check whether message 54 (the official
carrier) is itself gated by something in its own scheduling during early boot, separate from
`InsNavMode`'s own validity — inconclusive so far (monitoring started too late after a reset to
catch the actual early-boot window; both messages showed the same value at the same time once they
did appear).

**`nrx`→`urx` rename reverted (2026-07-23).** It broke the symmetry `app.py`/`nav_feed.py` depend
on — both had to learn to special-case which attribute name to use depending on which protocol was
active, purely because of a cosmetic rename in one file. Reverted `ucomrx_thread.py` back to `.nrx`
and removed the branching from both consumers entirely; `NcomRxThread`/`UcomRxThread` are fully
interchangeable again as far as anything outside `ucomrx_thread.py` is concerned.
