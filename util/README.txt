wireless-autonomous-terrestrial-navigation — Plymouth graphical boot theme
================================================

What it does
------------
- Centers your seal during boot.
- Fades it in.
- Adds a subtle, repeating radial pulse ring.
- Adds a subtle, repeating diagonal sheen sweep over the seal.

Install (root)
--------------
1) Copy theme into Plymouth themes directory:

   sudo cp -r wireless-autonomous-terrestrial-navigation /usr/share/plymouth/themes/

2) Set as default and rebuild initramfs:

   sudo plymouth-set-default-theme -R wireless-autonomous-terrestrial-navigation

Notes
-----
- If you do not see Plymouth at boot, ensure Plymouth is enabled and your initramfs includes it.
- On some systems you may need to explicitly enable Plymouth in your bootloader (e.g., GRUB: add 'splash').
