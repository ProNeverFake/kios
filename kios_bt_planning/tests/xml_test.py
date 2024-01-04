"""
BT in/out in XML format
"""

import xml.etree.ElementTree as ET


class BehaviorTreeFactory:
    def __init__(self):
        self.root = ET.Element("root", BTCPP_format="4")
        self.main_tree = ET.SubElement(self.root, "BehaviorTree", ID="MainTree")
        self.current_parent = ET.SubElement(
            self.main_tree, "ReactiveSequence", name="root_sequence"
        )
        self.subtrees = {}

    def add_node(self, node_type, name, **attributes):
        if node_type == "SubTree":
            subtree_id = attributes.get("ID")
            if subtree_id in self.subtrees:
                subtree_copy = self._deepcopy(self.subtrees[subtree_id])
                self.current_parent.append(subtree_copy)
            else:
                print(f"SubTree with ID {subtree_id} not found!")
                return
        else:
            node = ET.SubElement(
                self.current_parent, node_type, name=name, **attributes
            )
            if node_type in ["ReactiveSequence", "Fallback", "Sequence"]:
                self.current_parent = node

    def define_subtree(self, subtree_id, node_type, name):
        subtree = ET.Element(node_type, name=name)
        self.subtrees[subtree_id] = subtree
        return subtree

    def _deepcopy(self, element):
        """Creates a deep copy of an XML element and its children."""
        copy_elem = ET.Element(element.tag, attrib=element.attrib)
        for child in element:
            copy_elem.append(self._deepcopy(child))
        return copy_elem

    def to_string(self):
        return ET.tostring(self.root, encoding="utf-8", method="xml").decode("utf-8")

    def to_pretty_string(self):
        # For pretty printing
        import xml.dom.minidom

        xml_str = ET.tostring(self.root, encoding="utf-8", method="xml")
        dom = xml.dom.minidom.parseString(xml_str)
        return dom.toprettyxml()


# Usage
tree_factory = BehaviorTreeFactory()

tree_factory.add_node("CheckFinished", "check_finished")
tree_factory.add_node("SubTree", "", ID="push_peg_z")

# Peg align hole
align_subtree = tree_factory.add_node("ReactiveSequence", "peg_align_hole")
tree_factory.add_node("CheckAlign", "check_align")
tree_factory.add_node("Align", "align")

# Peg fit hole
fit_subtree = tree_factory.add_node("ReactiveSequence", "peg_fit_hole")
tree_factory.add_node("CheckFit", "check_fit")
tree_factory.add_node("Fit", "fit_hole")

# Peg reach hole
reach_subtree = tree_factory.add_node("ReactiveSequence", "peg_reach_hole")
tree_factory.add_node("CheckReach", "check_reach")
tree_factory.add_node("ReachHole", "reach_hole")

tree_factory.add_node("Approach", "approach")

# Defining the push_peg_z subtree
push_peg_subtree = tree_factory.define_subtree(
    "push_peg_z", "ReactiveSequence", "push_peg"
)
ET.SubElement(push_peg_subtree, "CheckPush", name="check_push")
ET.SubElement(push_peg_subtree, "Push", name="push_peg")

print(tree_factory.to_pretty_string())
