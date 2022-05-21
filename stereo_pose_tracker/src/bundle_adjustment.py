import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X, L

class stereoBundleAdjustment():
    def __init__(self, Ki, Kj, ext_params):

        self.Ki = gtsam.Cal3_S2(Ki[0,0], Ki[1,1], 0, Ki[0,-1], Ki[1,-1])
        self.Kj = gtsam.Cal3_S2(Kj[0,0], Kj[1,1], 0, Kj[0,-1], Kj[1,-1])
        self.camiNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.1, 0.1]) )
        self.camjNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.1, 0.1]) )
        
        self.X1_cov =  gtsam.noiseModel.Isotropic.Covariance(ext_params['X1_cov'])
        self.X2_cov =  gtsam.noiseModel.Isotropic.Covariance(ext_params['X2_cov'])
        self.X1 = gtsam.Pose3(ext_params['X1'])
        self.X2 = gtsam.Pose3(ext_params['X2'])     

    def addProjectionFactor(self, graph, K, z, noise, camera_key, landmark_key):
        self.F = gtsam.GenericProjectionFactorCal3_S2( z.reshape(2,1),noise, camera_key, landmark_key, K)
        graph.push_back(self.F)
    
    def run(self, pi, pj, geometrical_landmakrs):
            graph = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            num_of_landmarks = pi.shape[0]
            # In GTSAM, the pose assosiated with each camera is the transformation of the camera with respect the the world
            # However, the R and T that we extract using the geometrica method represent the transformation of the world (reference camera)
            # with respect to the camera (second camera). That is why in what follows, we feed the inverse of this transformation to the GTSAM
            graph.push_back(gtsam.PriorFactorPose3(X(0), self.X1, self.X1_cov))
            graph.push_back(gtsam.PriorFactorPose3(X(1), self.X2, self.X2_cov))
            initial.insert(X(0), self.X1)
            initial.insert(X(1), self.X2)

            for n in range(num_of_landmarks):
                self.addProjectionFactor(graph, self.Ki, pi[n,:], self.camiNoise, X(0), L(n))
                self.addProjectionFactor(graph, self.Kj, pj[n,:], self.camjNoise, X(1), L(n))
                initial.insert(L(n), gtsam.Point3(x=geometrical_landmakrs[n,0], 
                                                  y=geometrical_landmakrs[n,1], 
                                                  z=geometrical_landmakrs[n,2]))
#             params = gtsam.LevenbergMarquardtParams()
            params = gtsam.GaussNewtonParams()
#             optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
            result = optimizer.optimize()

            i = 0
            self.L = []
            #Extract the resutls (landmarks and poses)
            while result.exists(L(i)):
                self.L.append(result.atPoint3(L(i)))
                i += 1
            self.L = np.stack(self.L)