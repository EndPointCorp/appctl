"""
Helpers used by tests
"""

from exceptions import HelperException


class TestHelpers:
    @classmethod
    def filter_list_of_dicts(lyst, key, value):
        """
        filter dict from list of dicts - helpful for javascript objects
        [{"a": "b"},{"c": "d"},{...}] =filter=> [{
        """
        filtered_list = filter(lambda x: key in x.keys() and x[key] in value, lyst)

        if len(filtered_list) == 1:
            return filtered_list[0]
        else:
            raise HelperException("Error filtering list")
