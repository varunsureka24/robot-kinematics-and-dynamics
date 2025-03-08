def error(self, x0, R0, xf, Rf):
        R_diff = R0.T @ Rf

        # R_diff to axis angle, normalize, and get error per angle
        val = 0.5 * (R_diff[0, 0] + R_diff[1, 1] + R_diff[2, 2] - 1)
        if val > 1:
            val = 1
        elif val < -1:
            val = -1
        angle = np.arccos(val)
        if angle < 1e-12:
            return xf - x0, np.array([0, 0, 0])
        axis = 1/(2*np.sin(angle)) * np.array([R_diff[2, 1] - R_diff[1, 2], 
                                              R_diff[0, 2] - R_diff[2, 0],
                                              R_diff[1, 0] - R_diff[0, 1]])
        norm_axis = Rf @ (axis / np.linalg.norm(axis))
        rot_err = norm_axis * angle

        return xf - x0, rot_err

def twist(self, x0, R0, xf, Rf, dt):
    x_err, rot_err = self.error(x0, R0, xf, Rf)
    return x_err / dt, rot_err / dt
