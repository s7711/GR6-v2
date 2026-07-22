"""Publishes drive's live state over a Unix domain socket, for other
services on this machine to consume — the future `navigate`, `missions`,
a wheelspeed-GAD sender, `safety`. Same shape as `oxts-nav`'s
`nav_feed.py` — see drive-prd.md ("Feed naming") for why this is one
feed (`drive_feed`) covering motors/encoders/pump/ultrasonics together,
not split by concern.

Just a thin name for shared/feed_server.py's generic FeedServer — kept
as its own module/name since `drive/app.py` and this PRD already refer
to "DriveFeedServer" throughout.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.feed_server import FeedServer as DriveFeedServer  # noqa: E402,F401
