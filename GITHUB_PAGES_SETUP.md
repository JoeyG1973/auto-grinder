# GitHub Pages Setup Instructions

## Enabling GitHub Pages for Your Repository

After merging this pull request, follow these steps to enable GitHub Pages:

### 1. Navigate to Repository Settings
- Go to your repository: `https://github.com/JoeyG1973/auto-grinder`
- Click on the "Settings" tab (top right of the repository page)

### 2. Enable GitHub Pages
- Scroll down to the "Pages" section in the left sidebar
- Under "Source", select "Deploy from a branch"
- Choose the branch containing your files (typically `main`)
- Select the root folder "/ (root)"
- Click "Save"

### 3. Wait for Deployment
- GitHub Pages will build and deploy your site (usually takes 1-10 minutes)
- You'll see a green checkmark and URL when ready
- Your site will be available at: `https://joyg1973.github.io/auto-grinder/`

### 4. Test Your File Server
- Visit your GitHub Pages URL
- Test downloading files from the web interface
- Verify that direct file links work

### 5. Adding New Files
To add new files to your server:
1. Upload files to the `files/` directory
2. Commit and push changes
3. Files will be automatically available via your GitHub Pages site

### Troubleshooting
- If pages don't load, check the "Actions" tab for build errors
- Ensure all files are properly committed to the branch
- Large files (>100MB) are not supported by GitHub Pages

### Custom Domain (Optional)
- You can use a custom domain by adding a CNAME file
- See GitHub documentation for detailed instructions

---

**Your file server is now ready to use!** 🚀