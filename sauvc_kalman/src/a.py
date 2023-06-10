self.kf.B = np.array([[0.5*self.dt.secs*self.dt.secs, 0.5*self.dt.secs*self.dt.secs],                  ###
                                   [self.dt,self.dt]])

self.kf.F = np.array([[[1.,1.], self.dt],                                    
                                   [[0.,0.], 1.]])

 self.kf.H = np.array([[[1.,1.], 0.],
                             [[0.,0.], 1.]])

self.kf.x = np.dot(self.kf.F, self.kf.x) + np.dot(self.kf.B, u)