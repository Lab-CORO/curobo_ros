# Robot Integration

Documentation for integrating robots with curobo_ros.

## Supported Robots

### Fully Supported
- **[Doosan M1013](doosan-m1013.md)** - Doosan collaborative robot with full integration
  - Status: ✅ Production ready
  - Package: [curobo_doosan](https://github.com/Lab-CORO/curobo_doosan)
  - Features: Real robot control, emulator support, collision avoidance

### In Development
- **Universal Robots** - UR5e, UR10e, UR16e
  - Status: 🚧 In progress
  - Expected: Q1 2026

### Custom Integration
See [Tutorial: Adding Your Robot](../tutorials/02-adding-your-robot.md) for step-by-step integration guide.

## Integration Workflow

```
Robot URDF → Configuration YAML → Launch File → Testing → Production
```

See [Adding Your Robot Tutorial](../tutorials/02-adding-your-robot.md) for detailed steps.

```{toctree}
:maxdepth: 2
:hidden:

doosan-m1013
```
