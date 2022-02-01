# Fric parameter usage

```
res = []
for i in range(62, len(q_real)):
    q_r = q_real[i-62+1:i+1, :]
    qd_r = qd_real[i-62+1:i+1, :]
    residual = (u+G)[i-62:i, :] - torques_real[i-62+1:i+1, :]
    feature = np.concatenate([q_r, qd_r, residual], axis=1).reshape(1,-1)
    feature = np.concatenate([np.ones([1,1]), feature], axis=1)
    res.append(feature @ beta)
```

### 1 qd qdd [sin(kx) cos(kx)] for k=1 to 6
```
def feature(X):
	aug = X
	for k in range(1,7):
	    aug = np.concatenate([aug, np.sin(k * X)], axis=1)
	    aug = np.concatenate([aug, np.cos(k * X)], axis=1)
	poly = PolynomialFeatures(1)
	X = poly.fit_transform(aug)
	return X
```

### 1 q qd qdd [sin(kx) cos(kx)] for k=1 to 25
```
def feature(X):
	    aug = X
	    for k in range(1,26):
		aug = np.concatenate([aug, np.sin(k * X)], axis=1)
		aug = np.concatenate([aug, np.cos(k * X)], axis=1)

	    poly = PolynomialFeatures(1)
	    X = poly.fit_transform(aug)
	    return X
```
