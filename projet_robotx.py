import cv2
import numpy as np

def process_image(image):

    image = cv2.imread(image)

    # Definir la nouvelle largeur et hauteur souhaitees
    nouvelle_largeur = 600  # Remplacez par la largeur souhaitee
    nouvelle_hauteur = 600  # Remplacez par la hauteur souhaitee

    # Redimensionner l'image à la nouvelle taille
    image2 = cv2.resize(image, (nouvelle_largeur, nouvelle_hauteur))

    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Appliquer un flou gaussien pour reduire le bruit
    gray_blurred = cv2.GaussianBlur(gray, (15, 15), 0)

    # Detecter les contours avec l'operateur Canny
    edges1 = cv2.Canny(gray_blurred, threshold1=20, threshold2=60)





    # Trouver les coordonnees des points des contours
    contours, _ = cv2.findContours(edges1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Creer un nuage de points à partir des coordonnees des contours
    point_cloud = []
    for contour in contours:
        for point in contour:
            x, y = point[0]
            point_cloud.append((x, y))

    # Convertir la liste des points en un tableau numpy
    point_cloud = np.array(point_cloud)
    point_cloud = point_cloud[::8]


    # Extraire les coordonnees x et y du nuage de points
    x_coords = (point_cloud[:, 0]-300)*35/600
    y_coords = ((-1*point_cloud[:, 1])+600)*20/600


    """
    Partie cluster

    """

    X = point_cloud
    X[:,1] = -1 * X[:,1]
    X = np.asmatrix(X)
    m = X.shape[0]
    print(m)

    k = 7   #Nombre de cluster

    # ramdomly initialize mean points
    mu = X[np.random.randint(0, m, k), :]
    pre_mu = mu.copy()
    y = np.empty([m,1])

    # Run K-means
    for n_iter in range(500):
        for i in range(m):
            d0 = np.linalg.norm(X[i,:] - mu[0,:], 2)
            d1 = np.linalg.norm(X[i,:] - mu[1,:], 2)
            d2 = np.linalg.norm(X[i,:] - mu[2,:], 2)
            d3 = np.linalg.norm(X[i,:] - mu[3,:], 2)
            d4 = np.linalg.norm(X[i,:] - mu[4,:], 2)
            d5 = np.linalg.norm(X[i,:] - mu[5,:], 2)
            d6 = np.linalg.norm(X[i,:] - mu[6,:], 2)

            y[i] = np.argmin([d0, d1, d2, d3, d4, d5, d6])

        err = 0
        for i in range(k):
            mu[i,:] = np.mean(X[np.where(y == i)[0]], axis = 0)
            err += np.linalg.norm(pre_mu[i,:] - mu[i,:], 2)

        pre_mu = mu.copy()

        if err < 1e-10:
            print("Iteration:", n_iter)
            break

    X0 = X[np.where(y==0)[0]]
    X1 = X[np.where(y==1)[0]]
    X2 = X[np.where(y==2)[0]]
    X3 = X[np.where(y==3)[0]]
    X4 = X[np.where(y==4)[0]]
    X5 = X[np.where(y==5)[0]]
    X6 = X[np.where(y==6)[0]]


    tot = np.vstack((X0,X1,X2,X3,X4,X5,X6))

    # Separate points for each cluster
    cluster_points = [X[np.where(y == i)[0]] for i in range(k)]

    # List of points forming the face
    tota = np.vstack(cluster_points)

    # Calculate levels
    levels = [len(np.vstack(cluster_points[:i + 1])) for i in range(k)]


    Qx = (tota[:, 0])*20/60000 + 0.02
    Qy = (tota[:, 1]+300)*20/60000

    return Qx, Qy, levels