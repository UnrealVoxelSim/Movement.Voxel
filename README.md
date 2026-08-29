# UnrealVoxelSim.Movement.Voxel

Thread-affine deterministic grounded movement over solid voxels. `Controller` receives a capability-limited ECS access
object, queries the public movement and spatial components, and updates authoritative position, velocity, and grounded
state in place. It has no internal agent vector and cannot create entities or add/remove components.

Each update consumes `MovementInputComponent` only when its tick matches the current `StepContext`; stale input becomes
neutral. This implements the level-one side of the single-controller hierarchy: Navigation or another selected
level-two controller writes one desired velocity and jump request, and Movement alone applies gravity and collision.
Collision is a swept axis-aligned voxel test; out-of-bounds solid reads are treated as blocked.
