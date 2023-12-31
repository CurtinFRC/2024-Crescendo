# Contributing to our 2024 Codebase.
Our 2024 Codebase is split up into two main projects, our robot code and our library Wombat. This will only focus on our robot code but if you are on our programming team it is recomended that you also read the contributing guide for Wombat. This guide assumes you have already read the README.

## Git
Whenever you start on something new create a new branch from your master branch. The branch name should describe what it aims to achieve. When you are done you should open a pull request to CurtinFRC master.

## Git History
Try and keep git history clean. Commit messages should be *descriptive*.
An example of good Git usage would be as follows :
```bash
git add .
git commit -m '_commitmessage_' eg 'Fixed intake not holding in idle mode'
git push _remotename_ _branchname_ eg origin intake-fix
```

## Creating Issues
Issues should be created to report bugs or request features. Use the appropriate template for your issue and make sure to apply the appropriate labels. If you are working on an issue assign yourself to the issue.

## Creating Pull Requests
Pull requests should be created to resolve issues. Reference the issue you are resolving in your pull request. Pull requests are required to pass continuous integration checks that run automatically. This will include formatting, tests and build checks. Your pull request will automatically request a review from relevant teams.

## Wombat Contributions
Wombat contributions are expected to follow the guidelines set out in the contributing guide for Wombat. All Wombat changes during build season will be periodically reviewed and merged into a seperate branch of Wombat during build season. Outside of build season it will be less periodically merged in but will be directly merged to master. As Wombat is a 4788 published library there are stricter standards.

## Code Formatting
We have strict formatting rules to keep code consistent. We use the [WPILib styleguide](https://github.com/wpilibsuite/styleguide). In particular this means we use wpiformat. You can see [here](https://github.com/wpilibsuite/styleguide/blob/main/wpiformat/README.rst) how to install it. Before opening a pull request make sure to run wpiformat on the codebase.
