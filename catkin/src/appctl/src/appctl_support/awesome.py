# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import subprocess

AWESOME_CLIENT = '/usr/bin/awesome-client'


class WindowManager:
    """
    Interface for window show/hide.
    """
    def __init__(self, w_name=None, w_class=None, w_instance=None):
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance

    def _get_window_name_pattern(self):
        if self.w_name is not None:
            return 'name = "{}"'.format(self.w_name)
        else:
            return None

    def _get_window_class_pattern(self):
        if self.w_class is not None:
            return 'class = "{}"'.format(self.w_class)
        else:
            return None

    def _get_window_instance_pattern(self):
        if self.w_instance is not None:
            return 'instance = "{}"'.format(self.w_instance)
        else:
            return None

    def _get_rule_pattern(self):
        return ', '.join(
            filter(
                lambda w: w is not None, [
                    self._get_window_name_pattern(),
                    self._get_window_class_pattern(),
                    self._get_window_instance_pattern()
                ]
            )
        )

    def _get_properties(self, visible):
        return 'hidden = {}, minimized = {}, opacity = {}'.format(
            'false' if visible else 'true',
            'false' if visible else 'true',
            1 if visible else 0
        )

    def _get_script(self, visible):
        return ' '.join([
            self._removal_command(),
            'table.insert(awful.rules.rules, 1, {{ rule = {{ {rule} }}, properties = {{ {properties} }} }})'.format(
                rule = self._get_rule_pattern(),
                properties = self._get_properties(visible)
            ),
            'for k,c in pairs(client.get()) do if awful.rules.match(c, {{ {rule} }}) then awful.rules.apply(c) end end'.format(
                rule = self._get_rule_pattern()
            )
        ])

    def _get_command(self, visible):
        # TODO(mv): properly pipe to awesome-client
        return 'echo \'{}\' | {}'.format(self._get_script(visible), AWESOME_CLIENT)

    def _removal_command(self):
        def make_comparison(type, value):
            if value is not None:
                return 'rule["rule"]["{type}"] == "{value}"'.format(
                    type = type, value = value
                )
            else:
                return None

        comparison = ' and '.join(filter(lambda c: c is not None, [
            make_comparison('name', self.w_name),
            make_comparison('class', self.w_class),
            make_comparison('instance', self.w_instance)
        ]))
        return 'for key,rule in pairs(awful.rules.rules) do if {comparison} then table.remove(awful.rules.rules, key) end end'.format(
            comparison = comparison
        )

    def show(self):
        subprocess.check_call(self._get_command(True), shell=True)

    def hide(self):
        subprocess.check_call(self._get_command(False), shell=True)
