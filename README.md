# Physical AI & Humanoid Robotics Book

This project is a comprehensive educational resource for Physical AI & Humanoid Robotics, built with Docusaurus.

## Deployment to Vercel

This project is configured for deployment on Vercel. The configuration is already set up in the `vercel.json` file at the root of the repository.

### To deploy:

1. Connect this GitHub repository to Vercel
2. Vercel will automatically detect the Docusaurus project
3. The build command is set to `npm run build` which runs `cd physical-ai-book && docusaurus build`
4. Output directory is set to `physical-ai-book/build`

### Configuration Notes:
- The site is configured to deploy at: https://hackathon-project-beta-six.vercel.app
- The project uses the physical-ai-book directory as the main Docusaurus application
- All documentation is organized in modules under the docs directory

### Project Structure:
- `physical-ai-book/` - Main Docusaurus application
- `src/` - ROS2 modules and robotics code
- `simulation/` - Gazebo simulation files
- `specs/` - Project specifications

## Local Development

To run locally:
```bash
cd physical-ai-book
npm install
npm run start
```

## Vercel Deployment Status
This project is configured for deployment on Vercel and should be accessible at: https://hackathon-project-beta-six.vercel.app
