"""Unit test for hash_table.py module."""

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

from bt_learning.gp import hash_table


def test_hash_table_save_load():
    """Test saving and loading of hash table."""
    hash_table1 = hash_table.HashTable(size=3)
    hash_table1.insert(['1'], 1)
    hash_table1.insert(['2'], 2)
    hash_table1.insert(['3'], 3)
    hash_table1.insert(['4'], 4)
    hash_table1.insert(['4'], 5)
    hash_table1.insert(['5'], 5)
    hash_table1.insert(['6'], 5)
    hash_table1.insert(['7'], 5)
    hash_table1.write_table()
    hash_table2 = hash_table.HashTable(size=3)
    hash_table2.load()
    assert hash_table1 == hash_table2


def test_hash_table_eq():
    """Test hash table comparison function."""
    hash_table1 = hash_table.HashTable(size=10)
    hash_table2 = hash_table.HashTable(size=10)
    hash_table3 = hash_table.HashTable(size=10)
    hash_table1.insert(['1'], 1)
    hash_table1.insert(['2'], 2)
    hash_table2.insert(['3'], 3)
    hash_table2.insert(['4'], 4)
    hash_table3.insert(['1'], 1)
    hash_table3.insert(['2'], 2)

    assert hash_table1 == hash_table3
    assert hash_table1 != hash_table2
    assert hash_table1 != 1


def test_node_eq():
    """Test node comparison function."""
    node1 = hash_table.Node(['a'], 1)
    node2 = hash_table.Node(['a'], 1)
    node3 = hash_table.Node(['b'], 2)

    assert node1 == node2
    assert node1 != node3
    assert node1 != ['a']
    assert node1 != 1
    node1.next = node3
    assert node1 != node2
    node2.next = node3
    assert node1 == node2


def test_multiple_entries():
    """Test multiple entries for the same hash."""
    hash_table1 = hash_table.HashTable(size=10)
    hash_table1.insert(['a'], 1)
    hash_table1.insert(['a'], 2)
    hash_table1.insert(['a'], 3)
    hash_table1.insert(['b'], 4)
    hash_table1.insert(['b'], 5)
    hash_table1.insert(['b'], 6)
    assert hash_table1.find(['a']) == [1, 2, 3]
    assert hash_table1.find(['b']) == [4, 5, 6]


def test_to_string():
    """Test the to_string function."""
    assert hash_table.to_string(['a', 'b']) == 'a, b'

    assert hash_table.to_string(0) == '0'
