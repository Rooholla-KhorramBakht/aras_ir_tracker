import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X, L

class stereoBundleAdjustment():
    def __init__(self, Ki, Kj, ext_params):
        self.Ki = gtsam.Cal3_S2(Ki[0,0], Ki[1,1], 0, Ki[0,-1], Ki[1,-1])
        self.Kj = gtsam.Cal3_S2(Kj[0,0], Kj[1,1], 0, Kj[0,-1], Kj[1,-1])
        self.camiNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.1, 0.1]) )
        self.camjNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.01, 0.01]) )
        self.priorPoseNoise =  gtsam.noiseModel.Isotropic.Sigma(6,1e-7)
        self.relativePoseNoise = gtsam.noiseModel.Isotropic.Sigma(6,0.001)
        self.R_init = ext_params['R']
        self.t_init = ext_params['t']

    def addUnaryPosePrior(self, graph, T, noise, key):
        graph.push_back(gtsam.PriorFactorPose3(key, T, noise))         

    def addProjectionFactor(self, graph, K, z, noise, camera_key, landmark_key):
        self.F = gtsam.GenericProjectionFactorCal3_S2( z.reshape(2,1),noise, camera_key, landmark_key, K)
        graph.push_back(self.F)
        
    def addRelativeExtrinsics(self, graph, cam_i, cam_j, R, t, noise):
        Pose = gtsam.Pose3(gtsam.Rot3(R), t)
        graph.push_back(gtsam.BetweenFactorPose3(cam_i, cam_j, Pose, noise))
    
    def run(self, pi, pj, init_poses):
            graph = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            num_of_landmarks = pi.shape[0]
            # In GTSAM, the pose assosiated with each camera is the transformation of the camera with respect the the world
            # However, the R and T that we extract using the geometrica method represent the transformation of the world (reference camera)
            # with respect to the camera (second camera). That is why in what follows, we feed the inverse of this transformation to the GTSAM
            self.addUnaryPosePrior(graph, gtsam.Pose3(), self.priorPoseNoise, X(0))
#             self.addUnaryPosePrior(graph, gtsam.Pose3(gtsam.Rot3(self.R_init), self.t_init).inverse(), self.priorPoseNoise, X(1))

            initial.insert(X(0), gtsam.Pose3())
            initial.insert(X(1), gtsam.Pose3(gtsam.Rot3(self.R_init), self.t_init).inverse())
            
            #Add the transformation of the X(1) with respect to X(0) as a constraint to the graph
            self.addRelativeExtrinsics(graph, X(1), X(0), self.R_init, self.t_init, self.relativePoseNoise)

            for n in range(num_of_landmarks):
                self.addProjectionFactor(graph, self.Ki, pi[n,:], self.camiNoise, X(0), L(n))
                self.addProjectionFactor(graph, self.Kj, pj[n,:], self.camjNoise, X(1), L(n))
                initial.insert(L(n), gtsam.Point3(x=init_poses[n,0], y=init_poses[n,1], z=init_poses[n,2]))
#             print(self.F.error(initial))

#             params = gtsam.LevenbergMarquardtParams()
            params = gtsam.GaussNewtonParams()
#             optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
            result = optimizer.optimize()
            self.marginals = gtsam.Marginals(graph, result)

            i = 0
            self.L = []
            
#             print('Extrinsics Before:')
#             print(gtsam.Pose3(gtsam.Rot3(self.R_init.T), -self.R_init.T@self.t_init).matrix())
#             print('Extrinsics After:')
#             print(gtsam.Pose3(gtsam.Rot3(self.Rj), self.tj).matrix())
            #Extract the resutls (landmarks and poses)
            while result.exists(L(i)):
                self.L.append(result.atPoint3(L(i)))
                i += 1
            self.L = np.stack(self.L)