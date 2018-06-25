import unittest
from src.pit import TestSuite, TestCase, Report, InCategoryCondition, BinaryCondition


class MockTestCase(TestCase):
  def __init__(self, name):
    TestCase.__init__(self, name)
    self.ran = False

  def run(self):
    self.ran = True
    return Report(self)


VALID_CAT_FROM = 0
INVALID_CAT_FROM = 2


class ConcreteInCategoryCondition(InCategoryCondition):
  def __init__(self, name):
    InCategoryCondition.__init__(self, name)
    self.categories = set(["cat 1", "cat 2"])

  def measure(self, t):
    if t >= 0 and t < 1:
      return "cat 1"
    elif t >= 1 and t < 2:
      return "cat 2"
    else:
      return "cat 3"


EXPECT_FALSE_AT  = [0, 1]
EXPECT_TRUE_AT   = [2, 3] 
MEASURE_FALSE_AT = [0, 2]
MEASURE_TRUE_AT  = [1, 3]


class ConcreteBinaryCondition(BinaryCondition):
  '''
  Verifies correct at t == 0 and t == 3
  Verifies incorrect at t == 1 and t == 2
  '''
  def __init__(self, name):
    BinaryCondition.__init__(self, name)

  def expect_at(self, t):
    if t == 0 or t == 1:
      return False
    elif t == 2 or t == 3:
      return True

  def measure_at(self, t):
    if t == 0 or t == 2:
      return False
    elif t == 1 or t == 3:
      return True
    

class TestTestSuite(unittest.TestCase):
  def setUp(self):
    self.test_suite = TestSuite("Testing")

  def test_run_runs_all_test_cases(self):
    test_case_one = MockTestCase("Test case 1")
    test_case_two = MockTestCase("Test case 2")
    self.test_suite.test_cases.append(test_case_one)
    self.test_suite.test_cases.append(test_case_two)
    self.test_suite.run()
    self.assertTrue(test_case_one.ran)
    self.assertTrue(test_case_two.ran)

  def test_run_produces_reports(self):
    test_case_one = MockTestCase("Test case 1")
    test_case_two = MockTestCase("Test case 2")
    self.test_suite.test_cases.append(test_case_one)
    self.test_suite.test_cases.append(test_case_two)
    reports = self.test_suite.run()
    self.assertEqual(len(reports), 2)
    

class InCategoryConditionTests(unittest.TestCase):
  def setUp(self):
    self.condition = ConcreteInCategoryCondition("Test condition")

  def test_verify_verifies_true(self):
    self.assertTrue(self.condition.verify(VALID_CAT_FROM))

  def test_verify_verifies_false(self):
    self.assertFalse(self.condition.verify(INVALID_CAT_FROM))


class BinaryConditionTests(unittest.TestCase):
  def setUp(self):
    self.condition = ConcreteBinaryCondition("Test condition")

  def test_verify_verifies_if_expect_and_measure_both_true(self):
    self.assertTrue(self.condition.verify(3))

  def test_verify_verifies_if_expect_and_measure_both_false(self):
    self.assertTrue(self.condition.verify(0))
    
  def test_verify_verifies_false_if_expect_false_and_measure_true(self):
    self.assertFalse(self.condition.verify(1))

  def test_verify_verifies_false_if_expect_true_and_measure_false(self):
    self.assertFalse(self.condition.verify(2))
