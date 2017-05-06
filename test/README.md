# SMACHA Test Suite

Code for running diff tests to compare code generated by SMACHA to expected SMACH output.

## executive_smach_tutorials

These tests use a slightly modified subtree cloned from the
[executive_smach_tutorials repo](https://github.com/eacousineau/executive_smach_tutorials/tree/hydro-devel).
The code has been superficially refactored in places to make it easier to run diff tests.
E.g., class definition ordering may have been swapped to suit the recursion of the code generator, etc.