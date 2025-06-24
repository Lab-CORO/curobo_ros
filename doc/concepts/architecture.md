# Package architecture

## TODO class diagramm
---
config:
  theme: mc
  look: classic
  layout: elk
---
classDiagram
    class ConfigWrapperIK
    class IK
    class CuRoboTrajectoryMaker
    class ConfigWrapper
    class ConfigWrapperMotion
    class FK
    class JointCommandStrategy
    class RobotContext
    class GhostStrategy
    class DoosanControl
    class MarkerPublisher
    IK --> ConfigWrapperIK
    ConfigWrapper<--ConfigWrapperIK
    CuRoboTrajectoryMaker --> RobotContext
    RobotContext --> JointCommandStrategy
    JointCommandStrategy <|-- GhostStrategy
    JointCommandStrategy <|-- DoosanControl
    CuRoboTrajectoryMaker --> ConfigWrapperMotion
    ConfigWrapper<--ConfigWrapperMotion
    CuRoboTrajectoryMaker --> MarkerPublisher

## TODO ros node interface diageram