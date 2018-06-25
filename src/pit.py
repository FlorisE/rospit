import future
from abc import ABCMeta, abstractmethod

class TestSuite(object):
  __metaclass__ = ABCMeta

  def __init__(self, name):
    self.name = name
    self.test_cases = []

  def run(self):
    test_case_reports = []
    for test_case in self.test_cases:
      test_case_report = test_case.run()
      test_case_reports.append(test_case_report)
    return test_case_reports


class TestCase(object):
  __metaclass__ = ABCMeta

  def __init__(self, name, pre_conditions=[], invariants=[], post_conditions=[]):
    self.name = name
    self.pre_conditions = pre_conditions
    self.invariants = invariants
    self.post_conditions = post_conditions

  @abstractmethod
  def run(self): pass


class Report():
  def __init__(self, test_case, passed=True, pre_conditions=[], invariants=[], post_conditions=[]):
    self.test_case = test_case
    self.passed = passed
    self.pre_conditions = pre_conditions
    self.invariants = invariants
    self.post_conditions = post_conditions


class ConditionEvaluation():
  def __init__(self, failed=[], passed=[], violations=[]):
    self.failed = failed
    self.passed = passed
    self.violations = violations


class Violation():
  def __init__(self, actual, t, condition):
    self.actual = actual
    self.t = t
    self.condition = condition


class Condition(object):
  __metaclass__ = ABCMeta

  def __init__(self, name):
    self.name = name

  @abstractmethod
  def verify(self, t): pass


class Measurable(object):

  @abstractmethod
  def measure_at(self, t): pass


class InCategoryCondition(Condition, Measurable):
  def __init__(self, name, categories=set()):
    Condition.__init__(self, name)
    self.categories = categories

  def verify(self, t):
    return self.measure(t) in self.categories


class BinaryCondition(Condition, Measurable):
  def __init__(self, name):
    Condition.__init__(self, name)

  def verify(self, t):
    return self.measure_at(t) == self.expect_at(t)

  @abstractmethod
  def expect_at(self, t): pass


def ContinuousCondition(Condition, Measurable):
  def __init__(self, name, least_is_inclusive=True, most_is_inclusive=True):
    Condition.__init__(self, name)
    self.least_is_inclusive = least_is_inclusive
    self.most_is_inclusive = most_is_inclusive

  @abstractmethod
  def at_least(self, t): pass

  @abstractmethod
  def at_most(self, t): pass

  def verify(self, t):
    measured = measure(t)
    least_at_t = at_least(t)
    most_at_t = at_most(t)
    lower_than_most_at = measured <= most_at_t if self.most_is_inclusive else measured < most_at_t
    higher_than_least_at = measured >= least_at_t if self.least_is_inclusive else measured > least_at_t
    return lower_than_most_at and higher_than_least_at
