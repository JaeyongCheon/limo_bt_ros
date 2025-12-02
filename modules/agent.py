from modules.utils import config, env_pkg as utils_env_pkg, optional_import

if config is not None and isinstance(config, dict) and 'env_pkg' in config:
    env_pkg = config['env_pkg']
else:
    env_pkg = utils_env_pkg or "rescue_mission"

bt_module = optional_import(env_pkg + ".bt_nodes")
bt_module = optional_import(env_pkg + ".bt_nodes")

from modules.bt_constructor import build_behavior_tree
from modules.ros_bridge import ROSBridge

class Agent:
    def __init__(self, ros_namespace=None):
        self.blackboard = {}
        self.ros_bridge = ROSBridge.get()
        self.ros_namespace = ros_namespace      

    def create_behavior_tree(self, behavior_tree_xml):
        self.behavior_tree_xml = behavior_tree_xml
        self.tree = build_behavior_tree(self, behavior_tree_xml, env_pkg)

    def _reset_bt_action_node_status(self):
        self.tree.reset()

    async def run_tree(self):
        self._reset_bt_action_node_status()
        return await self.tree.run(self, self.blackboard)

