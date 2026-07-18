"""Minimal SSTG-Explorer example using the final algorithm configuration."""
from sstg_explorer import ExplorerConfig, SSTGExplorer
from sstg_explorer.environments import create_environment


def main():
    env = create_environment("maze", width=12.0, height=12.0)
    env.name = "maze"
    explorer = SSTGExplorer(config=ExplorerConfig(verbose=False))
    result = explorer.explore(env.get_occupancy_map(), env.get_start_pose())
    metadata = result["metadata"]
    print("SSTG-Explorer / maze")
    print(f"success:  {result['success']}")
    print(f"coverage: {metadata['coverage_ratio']:.2%}")
    print(f"distance: {metadata['total_distance']:.2f} m")
    print(f"nodes:    {len(result['nodes'])}")


if __name__ == "__main__":
    main()
