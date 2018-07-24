""" Tests for in category evaluators """

import unittest

from rospit.category import InCategoriesCondition, \
                            CategorySensor, \
                            CategoryMeasurement, \
                            InCategoryConditionEvaluator


valid_category = "valid_category"


class ValidCategorySensor(CategorySensor):
    """
    Returns a valid category
    """
    def sense(self):
        return CategoryMeasurement(valid_category)


invalid_category = "invalid_category"


class InvalidCategorySensor(CategorySensor):
    """
    Returns an invalid category
    """
    def sense(self):
        return CategoryMeasurement(invalid_category)


def evaluate_category_condition(in_categories):
    """
    Evaluates a category condition
    """
    condition = InCategoriesCondition([valid_category])
    if in_categories:
        sensor = ValidCategorySensor()
    else:
        sensor = InvalidCategorySensor()
    evaluator = InCategoryConditionEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


class InCategoryEvaluatorTest(unittest.TestCase):
    """
    Tests for in category evaluator
    """
    def test_evaluation_sets_the_condition(self):
        """
        Checks if the evaluator sets the condition
        """
        condition, _, _, evaluation = evaluate_category_condition(True)
        self.assertEqual(evaluation.condition, condition)

    def test_verifies_if_in_categories(self):
        """
        The evaluator should be nominal if the category is
        in the expected categories
        """
        _, _, _, evaluation = evaluate_category_condition(True)
        self.assertTrue(evaluation.nominal)

    def test_verifies_false_if_not_in_categories(self):
        """
        The evaluator should be nominal if the category is
        in the expected categories
        """
        _, _, _, evaluation = evaluate_category_condition(False)
        self.assertFalse(evaluation.nominal)
