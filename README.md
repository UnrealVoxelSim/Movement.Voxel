# UnrealVoxelSim.Movement.Voxel

Thread-affine deterministic grounded movement over solid voxels. Agents are stored in stable entity order, use Q32.32
continuous state, and consume one transient intent batch at a composition-defined update phase. Collision is a swept
axis-aligned voxel test; out-of-bounds solid reads are treated as blocked.
