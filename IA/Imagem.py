import cv2  # Importa a biblioteca OpenCV para processamento de imagem
import numpy as np  # Importa a biblioteca NumPy para manipulação de arrays

def adjust_brightness_contrast(image, brightness=0, contrast=0):
    # Ajusta o brilho e o contraste da imagem
    brightness = int((brightness - 0) * (255 - 0) / (100 - 0))  # Converte brilho para a escala de 0 a 255
    contrast = int((contrast - -127) * (127 - -127) / (100 - -100))  # Converte contraste para a escala de -127 a 127

    if contrast != 0:
        f = 131 * (contrast + 127) / (127 * (131 - contrast))  # Calcula o fator de ajuste de contraste
        image = cv2.addWeighted(image, f, image, 0, brightness - 127)  # Ajusta brilho e contraste
    else:
        image = cv2.add(image, brightness)  # Ajusta apenas o brilho
    return image

def resize_image(image, width=None, height=None):
    # Redimensiona a imagem mantendo a proporção original
    if width and height:
        return cv2.resize(image, (width, height))  # Redimensiona diretamente se largura e altura forem fornecidas
    elif width:
        ratio = width / image.shape[1]  # Calcula a proporção para redimensionamento
        height = int(image.shape[0] * ratio)  # Calcula a nova altura mantendo a proporção
        return cv2.resize(image, (width, height))  # Redimensiona a imagem
    elif height:
        ratio = height / image.shape[0]  # Calcula a proporção para redimensionamento
        width = int(image.shape[1] * ratio)  # Calcula a nova largura mantendo a proporção
        return cv2.resize(image, (width, height))  # Redimensiona a imagem
    else:
        return image  # Retorna a imagem original se nenhuma dimensão for fornecida

def rotate_image(image, angle):
    # Rotaciona a imagem em um determinado ângulo
    center = (image.shape[1] // 2, image.shape[0] // 2)  # Calcula o centro da imagem
    matrix = cv2.getRotationMatrix2D(center, angle, 1)  # Obtém a matriz de rotação
    return cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]))  # Aplica a rotação à imagem

def crop_image(image, x, y, w, h):
    # Corta a imagem para a região especificada
    return image[y:y+h, x:x+w]  # Retorna a região cortada da imagem

def apply_filter(image, kernel):
    # Aplica um filtro convolucional à imagem
    return cv2.filter2D(image, -1, kernel)  # Filtra a imagem usando o kernel fornecido

def segment_image(image, threshold):
    # Segmenta a imagem usando um limiar
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Converte a imagem para escala de cinza
    _, segmented = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)  # Aplica a limiarização
    return segmented  # Retorna a imagem segmentada

def equalize_histogram(image):
    # Equaliza o histograma da imagem para melhorar o contraste
    if len(image.shape) == 2:
        return cv2.equalizeHist(image)  # Equaliza histograma para imagens em escala de cinza
    elif len(image.shape) == 3 and image.shape[2] == 3:
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)  # Converte para o espaço de cores YCrCb
        ycrcb[:, :, 0] = cv2.equalizeHist(ycrcb[:, :, 0])  # Equaliza o canal Y (luminância)
        return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)  # Converte de volta para BGR

def main():
    # Caminho da imagem
    image_path = 'C:/Users/roger/AppData/Local/Microsoft/WindowsApps/IA/paisagem.jpg'
    image = cv2.imread(image_path)  # Carrega a imagem do caminho especificado

    if image is None:
        print(f"Erro ao carregar a imagem: {image_path}")
        return

    # Ajustar brilho e contraste
    image_bc = adjust_brightness_contrast(image, brightness=30, contrast=30)

    # Redimensionar
    image_resized = resize_image(image, width=200)

    # Rotacionar
    image_rotated = rotate_image(image, 45)

    # Cortar
    image_cropped = crop_image(image, 50, 50, 200, 200)

    # Filtrar
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    image_filtered = apply_filter(image, kernel)

    # Segmentar
    image_segmented = segment_image(image, 128)

    # Equalizar histograma
    image_equalized = equalize_histogram(image)

    # Mostrar resultados
    cv2.imshow('Original', image)
    cv2.imshow('Brilho e Contraste', image_bc)
    cv2.imshow('Redimensionado', image_resized)
    cv2.imshow('Rotacionado', image_rotated)
    cv2.imshow('Cortado', image_cropped)
    cv2.imshow('Filtrado', image_filtered)
    cv2.imshow('Segmentado', image_segmented)
    cv2.imshow('Equalizado', image_equalized)

    # Espera por uma tecla para fechar as janelas
    cv2.waitKey(0)
    cv2.destroyAllWindows()  # Fecha todas as janelas abertas

if __name__ == "__main__":
    main()  # Executa a função principal se o script for executado diretamente
