# Oriental Motor reference files

These vendor files are retained so the AZD3A-KED PDO configuration can be
reviewed and reproduced without relying on terminal output or browser history.

## Files

- `ORIENTALMOTOR_AZDxA-KED_rev0301.xml`
  - Official EtherCAT SubDevice Information (ESI) file.
  - Applies to AZD2A-KED, AZD3A-KED, and AZD4A-KED revision 0301.
  - Download:
    <https://www.orientalmotor.com/support/software/SFTWR/ORIENTALMOTOR_AZDxA-KED_rev0301.zip>
- `HM-60323-7E.pdf`
  - Official *AZ Series DC power input Multi-Axis Driver EtherCAT Compatible
    User Manual*.
  - Download:
    <https://www.orientalmotor.com/products/pdfs/opmanuals/HM-60323-7E.pdf>
- `SHA256SUMS`
  - SHA-256 checksums for detecting accidental file changes.

## Connected hardware identity

The live AZD3A-KED reported:

```text
Vendor ID:   0x000002BE
Product ID:  0x000013AF
Revision:    0x01110301
```

This selects the `AZD3A-KED rev0301` device entry in the ESI.

These files remain copyright Oriental Motor Co., Ltd. They are included here
as unmodified technical dependencies; consult the vendor website for updated
versions and applicable terms.
