"""
Unit test for gp_bt_interface.py
"""
import random
import pytest
import behavior_tree_learning.behavior_tree as behavior_tree
import behavior_tree_learning.gp_bt_interface as gp_bt_interface



def test_mutate_gene():
    """ Tests mutate_gene function """
    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')
    genome = ['s(', 'a0', ')']

    with pytest.raises(Exception):
        _ = gp_bt_interface.mutate_gene(genome, p_add=-1, p_delete=1)

    with pytest.raises(Exception):
        _ = gp_bt_interface.mutate_gene(genome, p_add=1, p_delete=1)

    for _ in range(10):
        #Loop many times to catch random errors
        mutated_genome = gp_bt_interface.mutate_gene(genome, p_add=1, p_delete=0)
        assert len(mutated_genome) >= len(genome)

        mutated_genome = gp_bt_interface.mutate_gene(genome, p_add=0, p_delete=1)
        assert len(mutated_genome) <= len(genome)

        mutated_genome = gp_bt_interface.mutate_gene(genome, p_add=0, p_delete=0)
        bt = behavior_tree.BT(mutated_genome)
        assert mutated_genome != genome
        assert bt.is_valid()

        mutated_genome = gp_bt_interface.mutate_gene(genome, p_add=0.3, p_delete=0.3)
        bt.set(mutated_genome)
        assert mutated_genome != genome
        assert bt.is_valid()

def test_crossover_genome():
    """ Tests crossover_genome function """
    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')
    genome1 = ['s(', 'c0', 'f(', 'c0', 'a0', ')', 'a0', ')']
    genome2 = ['f(', 'c1', 's(', 'c1', 'a1', ')', 'a1', ')']

    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2)

    assert offspring1 != []
    assert offspring2 != []
    assert offspring1 != genome1
    assert offspring1 != genome2
    assert offspring2 != genome1
    assert offspring2 != genome2

    bt1 = behavior_tree.BT(offspring1)
    assert bt1.is_valid()
    bt1 = bt1.set(offspring2)
    assert bt1.is_valid()

    genome1 = ['a0']
    genome2 = ['a1']
    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2)
    assert offspring1 == genome2
    assert offspring2 == genome1

    genome1 = []
    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2)
    assert offspring1 == []
    assert offspring2 == []

    for i in range(10):
        random.seed(i)
        offspring1, offspring2 = gp_bt_interface.crossover_genome(\
            gp_bt_interface.random_genome(10), gp_bt_interface.random_genome(10))
        bt1 = bt1.set(offspring1)
        assert bt1.is_valid()
        bt1 = bt1.set(offspring2)
        assert bt1.is_valid()

    genome1 = ['s(', 'f(', 'c0', 'a0', ')', 'a0', ')']
    genome2 = ['f(', 's(', 'c1', 'a1', ')', 'a1', ')']
    offspring1, offspring2 = gp_bt_interface.crossover_genome(genome1, genome2, replace=False)
    assert offspring1 != genome1
    assert offspring2 != genome2
    for gene in genome1:
        assert gene in offspring1
    for gene in genome2:
        assert gene in offspring2
