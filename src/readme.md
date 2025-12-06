# Maria-Carrillo-FIRST/src
## Place for production code used to build/run the robot.

Changes to these files should be reviewed in "pull requests" before being merged to the `main` branch.

To build the production code, open the desired robot project folder (for instance `src/swerve-baseline`) in "2025 WPILib VS Code".  If you opened a project directory and have the environment correctly configured, choosing "WPILib: Build Robot Code" from the command window in VSCode will succeed.

If this fails with a message like:
>  *  Executing task: gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" 
>
>'gradlew' is not recognized as an internal or external command,
>operable program or batch file.

The most likely problem is the wrong folder was opened in "2025 WPILib VS Code" (usually this is because the root repo folder is opened e.g. `c:\{...}\Maria-Carrillo-FIRST\` or `c:\{...}\Maria-Carrillo-FIRST\src` instead of the desired robot solution folder, e.g. `c:\{...}\Maria-Carrillo-FIRST\src\swerve-baseline` or `c:\{...}\Maria-Carrillo-FIRST\prototypes\autonomous-test`).

To work on a new subsystem or to experiment with code, do the following before starting:
# Create a branch in `git` from VSCode  
1. Click the `main` branch diagram in the status bar at the bottom of the screen and choose "Create new branch from..." from the command window.  
2. Choose what to branch from (likely `main` unless you are working on an integration branch with a partner).  
3. Name the new branch something short but descriptive for the intention of the work, like `prototype-fancydriving`.
# Create the baseline code to edit
1. Copy the current production robot baseline directory (currently `.\swerve-baseline`) to the prototypes folder, like `..\prototypes\swerve-baseline`.  
2. Rename the `swerve-baseline` directory to match the branch name, so renamed to something like `\..\prototypes\fancydriving`).  
3. Open the folder in "2025 WPILib VS Code".  
4. Verify the project builds using the command "WPILib: Build Robot Code".  
5. This is also a good time to confirm you can deploy and run the code successfully on the robot, __before making changes__. This will give you confidence to help troubleshoot problems in changed code later.  
# Start coding!
* Start experimenting, coding, and testing!  
* Build and test often (it makes finding and fixing bugs __much easier__)
* __Always keep safety in mind, Never assume code will behave as you expect, Always be ready to cut power__  
* Keep the robot on the development stand until basic functionality and safety have been proven on the development stand. 
# Check in your work!
* The git "Commit" is the fundamental way to mark a set of code changes. 
* The act of "Pushing" commits to github is what ensures your changes are available to others __and__ are backed up and protected from the inevitable computer mishap. 
* Commit and push changes to your own branches as often as you want.  Your branch is isolated from the production code and is your playground.  
# Merge back your changes...
* When you want to make your code changes easily accessible to others, create a github "pull request".  
* A "pull request" requires choosing a destination branch. This should be the branch you created your branch from (usually `main` branch, or integration branch if you are working with a partner). 
* A "pull request" then needs to be reviewed by one or more other team members before being approved for the merge. 
* This review process is important. This is where you share details about the design and coding choices you made, and is an opportunity for others to offer feedback.  
* Github offers an integrated "pull request" experience that promote code review conversations.

