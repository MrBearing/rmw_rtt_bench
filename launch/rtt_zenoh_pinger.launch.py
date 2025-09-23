from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Topics
    req_topic = LaunchConfiguration('req_topic', default='/latency_rtt_req')
    rep_topic = LaunchConfiguration('rep_topic', default='/latency_rtt_rep')

    # QoS
    qos_reliability = LaunchConfiguration('qos_reliability', default='best_effort')
    qos_history = LaunchConfiguration('qos_history', default='keep_last')
    qos_depth = LaunchConfiguration('qos_depth', default='10')

    # Pinger
    hz = LaunchConfiguration('hz', default='100.0')
    payload_size = LaunchConfiguration('payload_size', default='1024')
    duration_pinger = LaunchConfiguration('duration_pinger', default='30')
    trials = LaunchConfiguration('trials', default='-1')
    timeout = LaunchConfiguration('timeout', default='-1')
    csv = LaunchConfiguration('csv', default='results/rtt.csv')

    # Common
    intra_process = LaunchConfiguration('intra_process', default='false')
    transport_tag = LaunchConfiguration('transport_tag', default='')
    notes = LaunchConfiguration('notes', default='')
    domain_id = LaunchConfiguration('domain_id', default='42')
    start_router = LaunchConfiguration('start_router', default='false')
    router_listen = LaunchConfiguration('router_listen', default='')
    router_mode = LaunchConfiguration('router_mode', default='')

    def make_router(context):
        args = []
        listen = context.perform_substitution(router_listen)
        mode = context.perform_substitution(router_mode)
        if listen:
            args += ['--listen', listen]
        if mode:
            args += ['--mode', mode]
        return [Node(
            package='rmw_zenoh_cpp',
            executable='rmw_zenohd',
            name='rmw_zenohd',
            output='screen',
            condition=IfCondition(start_router),
            arguments=args,
        )]
    router = OpaqueFunction(function=make_router)

    pinger = Node(
        package='rmw_rtt_bench',
        executable='rtt_pinger',
        name='rtt_pinger',
        output='screen',
        arguments=[
            '--req-topic', req_topic,
            '--rep-topic', rep_topic,
            '--qos-reliability', qos_reliability,
            '--qos-history', qos_history,
            '--qos-depth', qos_depth,
            '--hz', hz,
            '--payload-size', payload_size,
            '--duration', duration_pinger,
            '--trials', trials,
            '--timeout', timeout,
            '--intra-process', intra_process,
            '--csv', csv,
            '--transport-tag', transport_tag,
            '--notes', notes,
        ],
    )

    return LaunchDescription([
        # Force zenoh RMW and domain id for both nodes
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),

        # Launch args
        DeclareLaunchArgument('req_topic', default_value=req_topic),
        DeclareLaunchArgument('rep_topic', default_value=rep_topic),
        DeclareLaunchArgument('qos_reliability', default_value=qos_reliability),
        DeclareLaunchArgument('qos_history', default_value=qos_history),
        DeclareLaunchArgument('qos_depth', default_value=qos_depth),
        DeclareLaunchArgument('hz', default_value=hz),
        DeclareLaunchArgument('payload_size', default_value=payload_size),
        DeclareLaunchArgument('duration_pinger', default_value=duration_pinger),
        DeclareLaunchArgument('trials', default_value=trials),
        DeclareLaunchArgument('timeout', default_value=timeout),
        DeclareLaunchArgument('csv', default_value=csv),
        DeclareLaunchArgument('intra_process', default_value=intra_process),
        DeclareLaunchArgument('transport_tag', default_value=transport_tag),
        DeclareLaunchArgument('notes', default_value=notes),
        DeclareLaunchArgument('domain_id', default_value=domain_id),
        DeclareLaunchArgument('start_router', default_value=start_router),
        DeclareLaunchArgument('router_listen', default_value=router_listen),
        DeclareLaunchArgument('router_mode', default_value=router_mode),

        router,
        pinger,
    ])
