# Project page: underactuated inverse kinematics (IJRR)

Static website, no build step. Open `index.html` in a browser to preview it locally.

## Publish on GitHub Pages (about 10 minutes)

1. In the group's GitHub organisation, create a new **public** repository, for example `underactuated-ik`.
2. Upload the whole content of this folder (`index.html`, `media/`, `img/`, `js/`, this README) to the repository root.
   On github.com: *Add file → Upload files*, drag the files and folders in, *Commit changes*.
3. Go to *Settings → Pages*. Under *Build and deployment*, choose *Deploy from a branch*, branch `main`, folder `/ (root)`, and *Save*.
4. After a minute the page is live at `https://<organisation>.github.io/underactuated-ik/`.

## Files

- `index.html`: the page.
- `js/playground.js`: interactive tendon-driven double pendulum (both strategies).
- `js/sheet3d.js`: interactive 3D view of the held configurations of a 3-joint arm.
- `media/`: explainer videos and loops (MP4, under 12 MB each), with poster frames.
- `img/`: block diagrams of the two strategies.

## Still to fill in

- Paper, arXiv and code links (buttons marked "link soon" in the header).
- The three simulation videos (dashed boxes in "Simulations of a soft arm").
- The MATLAB repository link (section "Implementation").
- Volume, pages and DOI in the BibTeX entry.
