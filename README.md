# Summary
SimpleControllers for Choreonoid.

# Usage
clone this repository in `choreonoid/ext` folder.
Don't forget `choreonoid/ext/CMakeLists.txt` before clone.

```bash
$ cd choreonoid/ext
$ rm CMakeLists.txt
$ git clone (this-repository-url) .
```

# Controllers
## PA10_JointAngleContrtoller
- PA10に関節空間上で指令値を与える

## PA10_JointTrajectoryController
- PA10に関節空間上で連続軌道を与える

## PA10_DrawCircleController
- PA10を作業空間での指令値で動かす