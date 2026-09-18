# Source (do not execute) before starting ROS processes that talk across the
# Intel–Thor Ethernet link, and in any shell that inspects Thor's topics:
#
#   source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh [intel|thor]
#
# Without an argument the role is chosen from the robot Ethernet address present
# on this host. Selects CycloneDDS with the host's Ethernet-only configuration;
# leaves ROS_DOMAIN_ID to the deployment. On Intel this replaces the robot
# services' Ethernet + Wi-Fi configuration from /etc/clearpath/setup.bash for
# this shell. See docs/physical/intel_thor_transport.md.

_dynamo_dds_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_dynamo_dds_role="${1:-}"
if [ -z "$_dynamo_dds_role" ]; then
    if ip -4 -o addr show | grep -q ' 192\.168\.131\.51/'; then
        _dynamo_dds_role=thor
    elif ip -4 -o addr show | grep -q ' 192\.168\.131\.1/'; then
        _dynamo_dds_role=intel
    fi
fi

case "$_dynamo_dds_role" in
    intel|thor)
        if [ -n "${RMW_IMPLEMENTATION:-}" ] && [ "$RMW_IMPLEMENTATION" != rmw_cyclonedds_cpp ]; then
            echo "dds_env.sh: replacing RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION with rmw_cyclonedds_cpp" >&2
        fi
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file://$_dynamo_dds_dir/cyclonedds_${_dynamo_dds_role}.xml"
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        echo "dds_env.sh: $_dynamo_dds_role configuration $CYCLONEDDS_URI" >&2
        ;;
    *)
        echo "dds_env.sh: not on the robot Ethernet; pass intel or thor explicitly" >&2
        unset _dynamo_dds_dir _dynamo_dds_role
        return 1
        ;;
esac
unset _dynamo_dds_dir _dynamo_dds_role
