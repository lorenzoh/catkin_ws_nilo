import numpy as np


def ransac(data, model, n_iterations, threshold, percentage_inliers):
    max_params = None
    max_consensus_size = 0

    for _ in range(n_iterations):

        subset = np.random.permutation(data)[: model.n_minimum_coeffs]
        params_subset = model.fit(subset)
        losses = model.loss(data, params_subset)
        consensus_set = data[losses < threshold]
        consensus_size = len(consensus_set)
        if (
            consensus_size / float(len(data)) < percentage_inliers
            or consensus_size < model.n_minimum_coeffs
        ):
            continue
        if consensus_size > max_consensus_size:
            max_consensus_size = consensus_size
            max_params = model.fit(consensus_set)

    if max_params is None:
        raise Exception("RANSAC: no estimator found with given constraints")
    else:
        return max_params



class LineModel:
    # The minimum number of coefficients needed to estimate a model
    n_minimum_coeffs = 20

    @classmethod
    def fit(cls, data):
        xs, ys = data[:, 1], data[:, 0]
        mx, my = np.mean(xs), np.mean(ys)

        variance = np.sum((xs - mx) * (ys - my))
        covariance = np.sum((xs - mx) ** 2)
        m = variance / covariance
        b = my - m * mx

        return m, b

    @classmethod
    def predict(cls, x, params):
        m, b = params
        return m * x + b

    @classmethod
    def loss(cls, data, params):
        """Calculate Mean Squared Error on `data` for `params = (m, b)`"""
        x = data[:, 1]
        y_true = data[:, 0]
        y_pred = cls.predict(x, params)
        return (y_true - y_pred) ** 2
        x = data[1]
        y_true = data[0]
        y_pred = cls.predict(x, params)
        return (y_true - y_pred) ** 2
