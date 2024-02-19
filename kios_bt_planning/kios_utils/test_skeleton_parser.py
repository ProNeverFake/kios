import unittest
from kios_utils.skeleton_parser import parse_node_name


class TestSkeletonParser(unittest.TestCase):
    def test_parse_node_name(self):
        # Test case 1: Valid input
        name = "BehaviorNode: ActionNode(param1, param2)"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": ["param1", "param2"],
        }
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 2: Empty input
        name = ""
        expected_output = {}
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 3: Invalid input
        name = "InvalidName"
        expected_output = {}
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 4: Input with missing parameters
        name = "BehaviorNode: ActionNode()"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": [],
        }
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 5: Input with single parameter
        name = "BehaviorNode: ActionNode(param1)"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": ["param1"],
        }
        self.assertEqual(parse_node_name(name), expected_output)


if __name__ == "__main__":
    unittest.main()

    def test_parse_node_name(self):
        # Test case 1: Valid input
        name = "BehaviorNode: ActionNode(param1, param2)"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": ["param1", "param2"],
        }
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 2: Empty input
        name = ""
        expected_output = {}
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 3: Invalid input
        name = "InvalidName"
        expected_output = {}
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 4: Input with missing parameters
        name = "BehaviorNode: ActionNode()"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": [],
        }
        self.assertEqual(parse_node_name(name), expected_output)

        # Test case 5: Input with single parameter
        name = "BehaviorNode: ActionNode(param1)"
        expected_output = {
            "typename": "BehaviorNode",
            "itemname": "ActionNode",
            "params": ["param1"],
        }
        self.assertEqual(parse_node_name(name), expected_output)
