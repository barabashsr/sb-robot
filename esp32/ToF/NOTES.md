### 07.03.2025.
1. For some reasons the refferencies to the pid setpoints stoped to work after micro ros implemented. In the BalanceController class they were changed from the privat vars directly to the params passed into constructor.

2. In order to get ros params server to work, I need to add .meta file for colcone config and add a propertie to the .ini file. Then run full clean of platform io.

3. Do not forget to run micro ros agent BEFORE running the micro ros. It causes the errors
