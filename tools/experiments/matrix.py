import numpy as np

plane1 = np.array[[1,-1,3,-1]][:,None]
plane2 = np.array[[1,2,-1,-3]][:,None]
## Matrix4d dp = pi1 * pi2.transpose[] - pi2 * pi1.transpose[];
# plk << dp[0,3], dp[1,3], dp[2,3], - dp[1,2], dp[0,2], - dp[0,1];

dp  = plane1 @ plane2.T - plane2 @ plane1.T
plk = [dp[0,3], dp[1,3], dp[2,3], - dp[1,2], dp[0,2], - dp[0,1]]

direction = np.cross[plane1[:3],plane2[:3]]
