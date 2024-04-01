# pylint: disable=comparison-with-itself, duplicate-code
"""
Unit test for hash_table.py
"""
import behavior_tree_learning.hash_table as hash_table

def test_hash_table_save_load():
    """ Tests saving and loading of hash table """
    hash_table1 = hash_table.HashTable(size=10)
    hash_table1.insert(['1'], 1)
    hash_table1.insert(['2'], 2)
    hash_table1.insert(['3'], 3)
    hash_table1.insert(['4'], 4)
    hash_table1.insert(['4'], 5)
    hash_table1.write_table()
    hash_table2 = hash_table.HashTable(size=10)
    hash_table2.load()
    assert hash_table1 == hash_table2

def test_hash_table_eq():
    """ Tests hash table comparison function """
    hash_table1 = hash_table.HashTable(size=10)
    hash_table2 = hash_table.HashTable(size=10)
    hash_table1.insert(['1'], 1)
    hash_table1.insert(['2'], 2)
    hash_table2.insert(['3'], 3)
    hash_table2.insert(['4'], 4)

    assert hash_table1 == hash_table1
    assert hash_table1 != hash_table2
    assert hash_table1 != 1

def test_node_eq():
    """ Tests node comparison function """
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
    """ Tests multiple entries for the same hash """
    hash_table1 = hash_table.HashTable(size=10)
    hash_table1.insert(['a'], 1)
    hash_table1.insert(['a'], 2)
    hash_table1.insert(['a'], 3)
    hash_table1.insert(['b'], 4)
    hash_table1.insert(['b'], 5)
    hash_table1.insert(['b'], 6)
    assert hash_table1.find(['a']) == [1, 2, 3]
    assert hash_table1.find(['b']) == [4, 5, 6]
