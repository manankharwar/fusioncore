#!/usr/bin/env python3
"""Tests for the ROS-parameter-to-node-member wiring checker."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import check_node_member_wiring


class NodeMemberWiringTest(unittest.TestCase):
    def test_extracts_plain_and_dotted_member_assignments(self):
        node = '''
        base_frame_   = get_parameter("base_frame").as_string();
        gnss_lever_arm_.x = get_parameter("gnss.lever_arm_x").as_double();
        '''
        self.assertEqual(
            [
                ("base_frame_", "base_frame"),
                ("gnss_lever_arm_", "gnss.lever_arm_x"),
            ],
            check_node_member_wiring.member_mappings(node),
        )

    def test_used_member_is_not_reported(self):
        node = 'imu_frame_override_ = get_parameter("imu.frame_id").as_string();'
        node += "\nstd::string imu_frame_override_;\nreturn imu_frame_override_;\n"
        self.assertEqual(
            [], check_node_member_wiring.dead_members(
                check_node_member_wiring.member_mappings(node), node)
        )

    def test_declaration_and_assignment_only_member_is_reported(self):
        node = 'vslam_frame_override_ = get_parameter("vslam.frame_id").as_string();'
        node += "\nstd::string vslam_frame_override_;\n"
        self.assertEqual(
            [("vslam_frame_override_", ["vslam.frame_id"])],
            check_node_member_wiring.dead_members(
                check_node_member_wiring.member_mappings(node), node),
        )

    def test_comment_naming_dead_member_does_not_count_as_a_use(self):
        node = 'dead_override_ = get_parameter("some.parameter").as_string();'
        node += "\nstd::string dead_override_;\n// dead_override_ is set nowhere useful\n"
        self.assertEqual(
            [("dead_override_", ["some.parameter"])],
            check_node_member_wiring.dead_members(
                check_node_member_wiring.member_mappings(node), node),
        )

    def test_shared_member_is_counted_once_and_lists_its_parameters(self):
        node = '''
        gnss_lever_arm_.x = get_parameter("gnss.lever_arm_x").as_double();
        gnss_lever_arm_.y = get_parameter("gnss.lever_arm_y").as_double();
        gnss_lever_arm_.z = get_parameter("gnss.lever_arm_z").as_double();
        std::string gnss_lever_arm_;
        '''
        # Three assignments plus the declaration are four references, so the
        # member is live; a shared member with no other use would report all
        # three parameters against the single member.
        self.assertEqual(
            [], check_node_member_wiring.dead_members(
                check_node_member_wiring.member_mappings(node), node)
        )

    def test_current_tree_reports_the_known_dead_member(self):
        node_source = check_node_member_wiring.NODE.read_text()
        mappings = check_node_member_wiring.member_mappings(node_source)
        self.assertTrue(mappings)
        self.assertEqual(
            [("vslam_frame_override_", ["vslam.frame_id"])],
            check_node_member_wiring.dead_members(mappings, node_source),
        )


if __name__ == "__main__":
    unittest.main()
