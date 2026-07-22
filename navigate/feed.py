"""Publishes navigate's live state (path-following status, tracked
position, cross-track error, distance travelled, clearance headroom)
over a Unix domain socket, for navigate's own web pages — same shape as
drive's drive_feed / oxts-nav's nav_feed. See navigate-prd.md
("Architecture / data flow").
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from shared.feed_server import FeedServer as NavigateFeedServer  # noqa: E402,F401
