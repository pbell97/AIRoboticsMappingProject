from PIL import Image


def createBlankMapArray(width, height):
    mapArray = []
    for i in range(width):
        tempList = []
        for j in range(height):
            tempList.append(0)
        mapArray.append(tempList)
    return mapArray


def pixelsToImage(pixelArray, imageName):
    width = len(pixelArray)
    height = len(pixelArray[0])

    newImage = Image.new('RGB', (width, height), color="White")
    pixels = newImage.load()

    for i in range(width):
        for j in range(height):
            if pixelArray[i][j]:
                pixels[i, j] = (0, 0, 0, 255)

    newImage.save(imageName)


if __name__ == "__main__":
    mapArray = createBlankMapArray(500, 500)
    for i in range(500):
        mapArray[300][i] = True
    pixelsToImage(mapArray, 'test3.png')
