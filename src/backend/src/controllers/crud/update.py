import tinydb


# Função para atualizar registro na base de dados com base no ID da mesma
# Necessário adicionar informações da versão, image e result em um JSON embutido

async def update(id, **kwargs):
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        # Atualiza apenas os campos fornecidos no kwargs
        db.update(kwargs, User.id == id)
        # Busca e retorna o registro atualizado para confirmar a mudança
        updated_record = db.search(User.id == id)
        db.close()
        if updated_record:
            return updated_record[0]
        else:
            return {"error": "No record found with the specified id"}
    except Exception as e:
        raise Exception(str(e))
