import sys

# Disable launch_testing plugins before pytest starts
pytest_plugins = []

def pytest_configure(config):
    # Remove the problematic plugins
    for plugin_name in ['launch_testing', 'launch_testing_ros', 'launch_testing_ros_pytest_entrypoint']:
        if plugin_name in config.pluginmanager.list_name_plugin():
            config.pluginmanager.unregister(name=plugin_name)
