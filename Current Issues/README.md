# Issue Tracking

This folder tracks issues discovered during development and testing of the Jetson Cube Orange Outdoor Rover.

## Structure

```
Current Issues/
├── README.md                    # This file
├── <ISSUE_NAME>.md              # Open issues (actively being worked on)
└── Resolved Issues/             # Completed issues (for reference)
    └── <RESOLVED_ISSUE>.md
```

## Issue Document Template

When creating a new issue, include:

```markdown
# Issue: [Brief Description]

**Date Reported:** [Date]
**Reported By:** [Who found it]
**Severity:** High/Medium/Low - [Impact description]

---

## Problem Description

[What's happening vs what should happen]

---

## Steps to Reproduce

1. [Step 1]
2. [Step 2]

---

## Root Cause (if known)

[Technical explanation]

---

## Suggested Fix

[Code or configuration changes needed]

---

## Files Affected

- `path/to/file.py`

---

**Status:** Open / In Progress / Resolved
**Assigned To:** [Development instance or person]
```

## When Resolved

1. Add a "## Resolution" section with:
   - Root cause explanation
   - Fix applied
   - Verification steps
2. Update status to "RESOLVED" with date
3. Move file to `Resolved Issues/` folder

## Current Open Issues

None

## Recently Resolved

| Issue | Date Resolved | Summary |
|-------|---------------|---------|
| DISARM_ENDPOINT_HANGING | Nov 21, 2025 | Service restart fixed stale state/executor deadlock |
| RTK_CORRECTIONS_STATUS_NOT_IN_API | Nov 21, 2025 | Added RTCM topic subscription to unified bridge |
| GPS_STATUS_NOT_RETURNING_DATA | Nov 21, 2025 | Service restart needed after MAVROS init |
| RTK_RTCM_FORWARDER_NOT_WORKING | Nov 21, 2025 | Updated MQTT IP + fixed QoS mismatch |
