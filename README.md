# Summary
SimpleControllers for Choreonoid.

# Usage
Clone [choreonoid](https://github.com/choreonoid/choreonoid) at any directory first.  
Then, clone this repository in `choreonoid/ext` folder.

> [!CAUTION]
> Don't forget `choreonoid/ext/CMakeLists.txt` before clone.

```bash
$ cd choreonoid/ext
$ rm CMakeLists.txt
$ git clone (this-repository-url) .
```

# Provided simple-controllers in this repository
## PA10_JointAngleContrtoller
- Provide random command values in the joint space to PA10.
- Change the joint angles at the following four time intervals during the simulation.
  - t1 = 0.0 ~ 2.5[s]
  - t2 = 2.5 ~ 5.0[s]
  - t3 = 5.0 ~ 7.5[s]
  - t4 = 7.5 ~ 10.0[s]