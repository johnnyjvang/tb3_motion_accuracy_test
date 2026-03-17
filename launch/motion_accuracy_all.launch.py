"""
motion_accuracy_all.launch.py

Runs TurtleBot3 motion accuracy tests sequentially
and prints a summary report at the end.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # Reset results file before starting tests
    reset_results = Node(
        package='tb3_motion_accuracy_test',
        executable='reset_results',
        output='screen'
    )

    # Test 1: out and back test
    out_and_back_test = Node(
        package='tb3_motion_accuracy_test',
        executable='out_and_back_test',
        output='screen'
    )

    # Test 2: square path test
    square_test = Node(
        package='tb3_motion_accuracy_test',
        executable='square_test',
        output='screen'
    )

    # Test 3: figure 8 test
    figure8_test = Node(
        package='tb3_motion_accuracy_test',
        executable='figure8_test',
        output='screen'
    )

    # Final summary report
    summary_report = Node(
        package='tb3_motion_accuracy_test',
        executable='summary_report',
        output='screen'
    )

    return LaunchDescription([

        # Start by resetting results file
        reset_results,

        # When reset_results exits, start out_and_back_test
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_results,
                on_exit=[TimerAction(period=1.0, actions=[out_and_back_test])]
            )
        ),

        # When out_and_back_test exits, start square_test
        RegisterEventHandler(
            OnProcessExit(
                target_action=out_and_back_test,
                on_exit=[TimerAction(period=1.0, actions=[square_test])]
            )
        ),

        # When square_test exits, start figure8_test
        RegisterEventHandler(
            OnProcessExit(
                target_action=square_test,
                on_exit=[TimerAction(period=1.0, actions=[figure8_test])]
            )
        ),

        # When figure8_test exits, start summary_report
        RegisterEventHandler(
            OnProcessExit(
                target_action=figure8_test,
                on_exit=[TimerAction(period=1.0, actions=[summary_report])]
            )
        ),

    ])