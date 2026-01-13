# Plymouth graphical boot theme

This directory packages the `wireless-autonomous-terrestrial-navigation` Plymouth theme and an
idempotent installer.

## What it does
- Centers your seal during boot.
- Fades it in.
- Adds a subtle, repeating radial pulse ring.
- Adds a subtle, repeating diagonal sheen sweep over the seal.

## Install (root)

Run the installer script to sync the theme into `/usr/share/plymouth/themes` and set it as the
current default if needed:

```bash
sudo ./install.sh
```

## Notes
- If you do not see Plymouth at boot, ensure Plymouth is enabled and your initramfs includes it.
- On some systems you may need to explicitly enable Plymouth in your bootloader (e.g., GRUB: add
  `splash`).
