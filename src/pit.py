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
      test_case_report = test_case.execute()
      test_case_reports.append(test_case_report)
    return test_case_reports


class TestCase(object):
  __metaclass__ = ABCMeta

  def __init__(self, name, preconditions=[], invariants=[], postconditions=[]):
    self.name = name
    self.preconditions = preconditions
    self.invariants = invariants
    self.postconditions = postconditions

  def verify_preconditions(self):
    return [c.evaluate() for c in self.preconditions]

  def verify_invariants(self, t):
    return [c.evaluate(t) for c in self.invariants]

  def verify_postconditions(self):
    return [c.evaluate() for c in self.postconditions]

  def execute(self):
    preconditions_evaluation = self.verify_preconditions()
    self.run()
    postconditions_evaluation = self.verify_postconditions()

  @abstractmethod
  def run(self): pass


class Report(object):
  def __init__(self, test_case, passed=True, preconditions=[], invariants=[], postconditions=[]):
    self.test_case = test_case
    self.passed = passed
    self.preconditions = preconditions
    self.invariants = invariants
    self.postconditions = postconditions


class ConditionEvaluation(object):
  def __init__(self, failed=[], passed=[], violations=[]):
    self.failed = failed
    self.passed = passed
    self.violations = violations


class Violation(object):
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
