# Handoff validation — 2026-10-02

Approved technical snapshot for source review:
`export/YuzuPeeler-MotorControl-review-20261002-v2`.
Archive: `YuzuPeeler-MotorControl-review-20261002.tar.gz`.
Do not send the entire export directory; older snapshots were retained.

## Repairs

- Captured the working multi-axis driver extension as a patch against the
  pinned upstream commit, including its C++ tests.
- Setup applies that patch and builds the application's dependency closure.
- ROS setup sourcing temporarily disables nounset. A minimal empty-environment
  shell reproduced the original unbound-variable failure and passes afterward.
- Startup helpers are executable and reject missing option values.
- Personal contact and local directory examples were replaced. The contact is
  explicitly a non-deliverable placeholder awaiting organizational approval.
- Export validation rejects personal paths and development artifacts and
  generates SHA256SUMS. Manufacturer XML contains uppercase HOME command names;
  path matching remains case-sensitive to avoid treating these as home paths.
- Supplied the already-declared MIT text and separate third-party notices.

## Checks completed

- 29 offline tests passed in both source and exported trees.
- Shell syntax and executable permissions checked.
- Driver patch applies to a clean local clone of the pinned commit; a second
  application is a no-op.
- All exported file checksums verified.
- No motor commands, live restarts or changes to the running workspace.

## Remaining deployment acceptance

The complete Internet-assisted install/build has not been executed on a clean
Ubuntu PC. Verify IgH kernel modules, target NIC, permissions, driver build,
GUI startup and staged machine motion on that PC before customer operation.
Offline tests do not certify machine safety. Obtain the owner's release/contact
approval and confirm manufacturer-document redistribution terms before sharing.
