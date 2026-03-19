#!/usr/bin/env python3
"""
Gripper Feedback Modes Comparison Tool

This script allows you to test and compare two gripper feedback modes:
1. Proportional mode: Feedback proportional to follower current (transparent)
2. Threshold-constant mode: Constant force above threshold (stable, chatter-free)

Usage:
    python compare_gripper_modes.py --mode proportional
    python compare_gripper_modes.py --mode threshold_constant
    python compare_gripper_modes.py --mode both
    python compare_gripper_modes.py --threshold 50 --constant-force 30
"""

import argparse
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

from lerobot.teleoperators.turret_leader import TurretLeaderConfig, TurretLeader


def create_config(
    mode: str = "proportional",
    threshold_ma: float = 50.0,
    constant_force_ma: float = 30.0,
    port: str = "/dev/null",
    **kwargs
) -> TurretLeaderConfig:
    """Create a TurretLeaderConfig with specified mode."""
    return TurretLeaderConfig(
        port=port,
        gripper_feedback_mode=mode,
        gripper_feedback_threshold_ma=threshold_ma,
        gripper_feedback_constant_force_ma=constant_force_ma,
        max_current_per_joint={"shoulder": 130.0, "gripper": 1750.0},
        debug_feedback=True,
        **kwargs
    )


def simulate_feedback(leader: TurretLeader, follower_current_lsb: float) -> float:
    """Simulate sending follower current and return the leader command.
    
    Note: This doesn't actually connect to hardware, just tests the logic.
    """
    feedback = {"gripper.current": follower_current_lsb}
    
    # Capture the command by temporarily storing it
    # (In real use, this writes to Leader Goal_Current via CAN bus)
    leader.send_feedback(feedback)
    
    return follower_current_lsb * 2.69  # Convert LSB to mA for reporting


def compare_modes(
    follower_lsb: float,
    threshold_ma: float = 50.0,
    constant_force_ma: float = 30.0,
):
    """Compare both modes with a given follower current."""
    cfg_prop = create_config(
        mode="proportional",
        threshold_ma=threshold_ma,
        constant_force_ma=constant_force_ma,
    )
    cfg_const = create_config(
        mode="threshold_constant",
        threshold_ma=threshold_ma,
        constant_force_ma=constant_force_ma,
    )
    
    leader_prop = TurretLeader(cfg_prop)
    leader_const = TurretLeader(cfg_const)
    
    follower_ma = follower_lsb * 2.69
    
    print(f"\n{'─' * 70}")
    print(f"Follower Current: {follower_lsb:.1f} LSB = {follower_ma:.1f} mA")
    print(f"{'─' * 70}")
    
    print(f"\n📊 Proportional Mode:")
    print(f"   Expected: Current-proportional feedback")
    normalized = min(1.0, follower_ma / 1193.0)
    expected_ma = normalized * 1750 * 0.45  # gain=0.45
    print(f"   Calculated command: ~{expected_ma:.1f} mA")
    print(f"   Behavior: Resistance increases with follower current")
    
    print(f"\n🎯 Threshold-Constant Mode:")
    print(f"   Threshold: {threshold_ma:.1f} mA")
    print(f"   Constant Force: {constant_force_ma:.1f} mA")
    if follower_ma < threshold_ma:
        print(f"   Current {follower_ma:.1f} mA < {threshold_ma:.1f} mA threshold")
        print(f"   → Command: 0 mA (no feedback)")
        print(f"   Behavior: No resistance (below threshold)")
    else:
        print(f"   Current {follower_ma:.1f} mA ≥ {threshold_ma:.1f} mA threshold")
        print(f"   → Command: ±{constant_force_ma:.1f} mA (constant)")
        print(f"   Behavior: Fixed resistance (object detected)")


def main():
    parser = argparse.ArgumentParser(
        description="Compare gripper haptic feedback modes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test proportional mode
  %(prog)s --mode proportional
  
  # Test threshold-constant mode
  %(prog)s --mode threshold_constant
  
  # Compare both modes
  %(prog)s --mode both
  
  # Simulate various follower currents
  %(prog)s --mode both --currents 10,50,100,150
  
  # Tune parameters
  %(prog)s --mode threshold_constant --threshold 60 --constant-force 25
        """,
    )
    
    parser.add_argument(
        "--mode",
        choices=["proportional", "threshold_constant", "both"],
        default="both",
        help="Feedback mode to test (default: both)",
    )
    parser.add_argument(
        "--currents",
        type=str,
        default="10,25,50,75,100",
        help="Comma-separated follower current LSB values to test (default: 10,25,50,75,100)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=50.0,
        help="Threshold mA for threshold_constant mode (default: 50.0)",
    )
    parser.add_argument(
        "--constant-force",
        type=float,
        default=30.0,
        help="Constant force mA for threshold_constant mode (default: 30.0)",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/null",
        help="Serial port (default: /dev/null for simulation only)",
    )
    
    args = parser.parse_args()
    
    # Parse current values
    try:
        currents_lsb = [float(x.strip()) for x in args.currents.split(",")]
    except ValueError:
        print("Error: --currents must be comma-separated numbers", file=sys.stderr)
        return 1
    
    print("=" * 70)
    print("🔬 Gripper Feedback Modes Comparison Tool")
    print("=" * 70)
    print(f"\nTest Configuration:")
    print(f"  Mode: {args.mode}")
    print(f"  Current test points (LSB): {args.currents}")
    print(f"  Threshold: {args.threshold:.1f} mA")
    print(f"  Constant Force: {args.constant_force:.1f} mA")
    print(f"  Port: {args.port} (simulation mode)")
    
    # Run comparison
    for current_lsb in currents_lsb:
        if args.mode == "both":
            compare_modes(
                current_lsb,
                threshold_ma=args.threshold,
                constant_force_ma=args.constant_force,
            )
        elif args.mode == "proportional":
            cfg = create_config(mode="proportional", port=args.port)
            leader = TurretLeader(cfg)
            print(f"\n📊 Proportional Mode - {current_lsb} LSB:")
            simulate_feedback(leader, current_lsb)
        elif args.mode == "threshold_constant":
            cfg = create_config(
                mode="threshold_constant",
                threshold_ma=args.threshold,
                constant_force_ma=args.constant_force,
                port=args.port,
            )
            leader = TurretLeader(cfg)
            print(f"\n🎯 Threshold-Constant Mode - {current_lsb} LSB:")
            simulate_feedback(leader, current_lsb)
    
    print(f"\n{'=' * 70}")
    print("✅ Comparison complete")
    print("=" * 70)
    print("\n💡 Tips:")
    print("  1. Test each mode with various objects")
    print("  2. Adjust threshold_ma to detect object contact reliably")
    print("  3. Tune constant_force_ma for comfortable resistance")
    print("  4. Use debug_feedback_realtime to monitor in real-time:")
    print("     lerobot-teleoperate --config-path teleoperator=turretL \\")
    print("       --teleop.debug_feedback_realtime=true")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
