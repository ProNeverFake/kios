"""Custom YAML dumper for dictionaries."""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import yaml


def represent_bool_custom(dumper: yaml.Dumper, data: bool):
    """
    Represent bool data.

    Equivalent to the original function, just with capitalized initials.
    """
    if data:
        value = 'True'
    else:
        value = 'False'
    return yaml.representer.BaseRepresenter.represent_scalar(
        dumper, 'tag:yaml.org,2002:bool', value)


def represent_dict_custom(dumper: yaml.Dumper, dictionary: dict):
    """
    Represent dictionary data.

    - Keys are treated as in the original representer.
    - Bool values are treated according to the modified bool representer.
    - String values are single quoted.
    - Other types of values are treated by default.
    """
    value = []
    node = yaml.MappingNode('tag:yaml.org,2002:map', value)

    for item_key in dictionary:
        item_value = dictionary[item_key]
        node_key = yaml.representer.BaseRepresenter.represent_data(dumper, item_key)
        if type(item_value) == str:
            node_value = yaml.representer.BaseRepresenter.represent_scalar(
                dumper, 'tag:yaml.org,2002:str', item_value, style="'")
        elif type(item_value) == bool:
            node_value = represent_bool_custom(dumper, item_value)
        elif type(item_value) == list:
            node_value = yaml.representer.BaseRepresenter.represent_sequence(
                dumper, 'tag:yaml.org,2002:seq', item_value)
        else:
            node_value = yaml.representer.BaseRepresenter.represent_data(dumper, item_value)

        value.append((node_key, node_value))

    return node


def add_custom_dumper():
    """Add the custom dumper to the Dumper class."""
    yaml.add_representer(dict, represent_dict_custom)
