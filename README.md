# Fric parameter usage

```)
res = []
for i in range(62, len(q_real)):
    q_r = q_real[i-62+1:i+1, :]
    qd_r = qd_real[i-62+1:i+1, :]
    residual = (u+G)[i-62:i, :] - torques_real[i-62+1:i+1, :]
    feature = np.concatenate([q_r, qd_r, residual], axis=1).reshape(1,-1)
    feature = np.concatenate([np.ones([1,1]), feature], axis=1)
    res.append(feature @ beta)
(```
