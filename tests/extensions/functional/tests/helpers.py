"""
Helpers used by tests
"""

from exception import HelperException


class TestHelpers:
    def filter_list_of_dicts(self, lyst, key, value):
        """
        filter dict from list of two value dicts - helpful for javascript objects
        when you've got "X" and "Y"
        [{"Description": "X", "Value": "Y"},{...}
        """
        filtered_list = filter(lambda x: key in x.values() and value in x.values(), lyst)

        if len(filtered_list) == 1:
            return True
        else:
            raise HelperException("Error filtering list")
