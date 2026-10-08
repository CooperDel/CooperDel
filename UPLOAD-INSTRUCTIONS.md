# Uploading this optimized copy

This folder is the deployment copy. Original files in the Portfolio folder are unchanged.

Create an empty public GitHub repository named YOUR-USERNAME.github.io, then run these commands in PowerShell, replacing YOUR-USERNAME:

```powershell
cd C:\Users\coope\Desktop\Portfolio\github-upload
git init
git config --global --add safe.directory C:/Users/coope/Desktop/Portfolio/github-upload
git add .
git commit -m "Publish optimized portfolio"
git branch -M main
git remote add origin https://github.com/YOUR-USERNAME/YOUR-USERNAME.github.io.git
git push -u origin main
```

In the repository, open Settings → Pages. Choose Deploy from a branch, main, and / (root), then Save.

The .gitignore excludes local optimization tools. Keep .nojekyll and all published asset folders. Future source edits need to be rebuilt and optimized into a new deployment copy; this folder is a snapshot.
