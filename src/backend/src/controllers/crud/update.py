import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

# Função para atualizar registro na base de dados com base no ID da mesma
# Necessário adicionar informações da versão, image e result em um JSON embutido

async def update(id, version, image, result):
    try:
        db.update({'version': version, 'image': image, 'result': result}, User.id == id)
        return {"id": id, "version": version, "image": image, "result": result}
    except Exception as e:
        raise Exception(str(e))
