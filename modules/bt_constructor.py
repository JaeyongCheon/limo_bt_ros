import os
from modules.utils import (
    parse_behavior_tree,
    convert_value,
    get_file_dirname,
    optional_import,
)


def build_behavior_tree(agent, behavior_tree_xml: str, env_pkg: str):
    """
    Build a Behavior Tree from an XML file for the given agent and environment.

    Parameters
    ----------
    agent : Agent (or compatible)
        Agent instance; passed to action/condition constructors.
    behavior_tree_xml : str
        Path to XML <BehaviorTree>.
    env_pkg : str
        Dotted package path (e.g., "scenarios.rescue_mission")
    """

    bt_module = optional_import(f"{env_pkg}.bt_nodes")
    mission_bt_module = optional_import(f"{env_pkg}.mission_bt_nodes")

    if bt_module is None:
        raise ModuleNotFoundError(
            f"[ERROR] Could not import '{env_pkg}.bt_nodes'. "
            "Make sure your environment package exposes bt_nodes."
        )

    xml_root = parse_behavior_tree(behavior_tree_xml)

    return _parse_xml_to_bt(
        xml_root.find("BehaviorTree"),
        bt_module=bt_module,
        mission_bt_module=mission_bt_module,
        agent=agent,
        top_xml_path=behavior_tree_xml,
    )


def _parse_xml_to_bt(xml_node, *, bt_module, mission_bt_module, agent, top_xml_path):
    node_type = xml_node.tag

    # ---------- SubTree support ----------
    if node_type == "SubTree":
        subtree_id = xml_node.attrib.get("ID")
        if not subtree_id:
            raise ValueError("[ERROR] SubTree node must have an 'ID' attribute")

        base_dir = get_file_dirname(top_xml_path)
        sub_behavior_tree_xml = os.path.join(base_dir, f"{subtree_id}.xml")
        subtree_root = parse_behavior_tree(sub_behavior_tree_xml)

        return _parse_xml_to_bt(
            subtree_root.find("BehaviorTree"),
            bt_module=bt_module,
            mission_bt_module=mission_bt_module,
            agent=agent,
            top_xml_path=sub_behavior_tree_xml,
        )

    # ---------- Recursively build children ----------
    children = [
        _parse_xml_to_bt(
            child,
            bt_module=bt_module,
            mission_bt_module=mission_bt_module,
            agent=agent,
            top_xml_path=top_xml_path,
        )
        for child in xml_node
    ]

    BTNodeList = getattr(bt_module, "BTNodeList")

    # ---------- Convert XML attributes ----------
    attrib = {k: convert_value(v) for k, v in xml_node.attrib.items()}

    # ===========================================================
    # CONTROL NODES  (Sequence, Selector, etc.)
    # ===========================================================
    if node_type in BTNodeList.CONTROL_NODES:
        control_class = getattr(bt_module, node_type)

        # Remove duplicate 'name' parameter: node_type already fills it
        attrib = dict(attrib)
        attrib.pop("name", None)

        return control_class(node_type, children=children, **attrib)

    # ===========================================================
    # DECORATOR NODES  (Inverter, Retry, etc.)
    # ===========================================================
    elif node_type in BTNodeList.DECORATOR_NODES:
        decorator_class = getattr(bt_module, node_type)

        if len(children) != 1:
            raise ValueError(
                f"[ERROR] Decorator '{node_type}' must have exactly one child."
            )

        return decorator_class(node_type, child=children[0], **attrib)

    # ===========================================================
    # ACTION / CONDITION NODES
    # ===========================================================
    elif node_type in (BTNodeList.ACTION_NODES + BTNodeList.CONDITION_NODES):
        action_class = getattr(bt_module, node_type)
        return action_class(node_type, agent, **attrib)

    # ===========================================================
    # ROOT BehaviorTree tag
    # ===========================================================
    elif node_type == "BehaviorTree":
        if not children:
            raise ValueError("[ERROR] <BehaviorTree> has no child node.")
        return children[0]

    else:
        raise ValueError(f"[ERROR] Unknown behavior node type: {node_type}")
