# Fric parameter usage


### 1 q dq [sin(kx) cos(kx)] for k=1 to 25 (1 + 2 + 4 * 25)
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


### 1 e dot(e) [] [sin(kx) cos(kx)] for k=1 to 6 (1 + 14 + 28 * 6)
```
def feature_tri(x):
        aug = x
        for k in range(1,7):
            aug = np.concatenate([aug, np.sin(x)], axis=1)
            aug = np.concatenate([aug, np.cos(x)], axis=1)
        poly = PolynomialFeatures(1)
        aug = poly.fit_transform(aug)
        return aug
```

### NN weight: input 14 (q, dq) or (e dot(e)) -> 4 or 3
```
MLP(
  (net): Sequential(
    (0): Linear(in_features=14, out_features=28, bias=True)
    (1): ReLU()
    (2): Linear(in_features=28, out_features=14, bias=True)
    (3): ReLU()
    (4): Linear(in_features=14, out_features=4, bias=True)
  )
)
```
