---
sidebar_position: 2
custom_edit_url: null
title: "Banco de Dados"
---
# Banco de Dados

## Estrutura da Base de Dados

Estamos utilizando o TinyDB como nossa base de dados para armazenar as informações das imagens processadas pela nossa aplicação. A estrutura do banco de dados é conforme descrito abaixo:

### Campos da Base de Dados

- **id:** Identificador único para cada entrada no banco de dados.
- **version:** Versão da imagem processada.
- **image:** String da imagem codificada em base64.
- **result:** Resultado da identificação de sujeiras na imagem pela I.A. (`True` se sujeira foi identificada, `False` caso contrário).

### Exemplo de Entrada no Banco de Dados

```json
{
  "id": 2,
  "version": "v1.0",
  "image": "imageBase64String",
  "result": "True"
}
```

### Detalhes dos Campos

- **id:** Este campo é um número inteiro que serve como identificador único para cada registro na base de dados. Ele garante que cada imagem processada possa ser referenciada e recuperada de forma única. Isso é especialmente importante em sistemas onde múltiplas entradas são criadas frequentemente.
- **version:** Este campo armazena a versão da imagem processada. Pode ser útil para rastrear mudanças ou atualizações nas imagens, permitindo uma visão histórica das versões das imagens e das melhorias nas técnicas de processamento ao longo do tempo.
- **image:** Este campo contém a representação da imagem em uma string codificada em base64. A codificação base64 é uma forma de converter dados binários em uma string ASCII, facilitando o armazenamento e a transmissão de dados que contêm caracteres não imprimíveis. Isso é especialmente útil para imagens, que são normalmente armazenadas em formatos binários.
- **result:** Este campo armazena o resultado da análise da imagem pela inteligência artificial. Ele pode ter dois valores:
    - **`True`:** Indica que sujeira foi identificada na imagem.
    - **`False`:** Indica que nenhuma sujeira foi identificada na imagem.
    A existência deste campo é fundamental para a lógica da nossa aplicação, pois permite determinar se a análise automática foi bem-sucedida ou não.

## Utilizando o TinyDB

### Inserindo dados

Para inserir dados no TinyDB, utilizamos o seguinte código Python. Esse código demonstra como criar uma nova entrada no banco de dados com todas as informações necessárias sobre uma imagem processada:

```python
from tinydb import TinyDB, Query

db = TinyDB('db.json')

# Dados a serem inseridos
data = {
    "id": 2,
    "version": "v1.0",
    "image": "imageBase64String",
    "result": True
}

# Inserindo os dados
db.insert(data)
```

### Recuperando dados

Para recuperar/ler dados do TinyDB, podemos usar o seguinte código Python. Esse código mostra como obter todos os registros do banco de dados ou buscar um registro específico usando o campo `id`:

```python
from tinydb import TinyDB, Query

db = TinyDB('db.json')

# Recuperar todos os dados
all_data = db.all()

# Recuperar dados específicos
query = Query()
result = db.search(query.id == 2)
```

### Atualizando dados

Para atualizar um registro existente no TinyDB, utilizamos o código abaixo. Este exemplo demonstra como atualizar o campo `result` de um registro específico identificado pelo `id`:

```python
from tinydb import TinyDB, Query

db = TinyDB('db.json')

# Dados a serem atualizados
query = Query()
db.update({"result": False}, query.id == 2)
```

### Excluindo dados

Para excluir um registro do TinyDB, usamos o seguinte código. Este exemplo mostra como remover um registro específico do banco de dados usando o campo `id` como identificador:

```python
from tinydb import TinyDB, Query

db = TinyDB('db.json')

# Excluir dados
query = Query()
db.remove(query.id == 2)
```

## Conclusão

O TinyDB é uma excelente escolha para aplicações que necessitam de uma base de dados simples, flexível e fácil de usar. Ele permite armazenar dados estruturados de forma eficiente em um arquivo JSON, facilitando a implementação e a manutenção de aplicações pequenas a médias.

Com a estrutura descrita acima, podemos armazenar eficientemente as imagens processadas e os resultados das análises realizadas pela nossa inteligência artificial. Essa abordagem simplifica o desenvolvimento e a gestão dos dados, ao mesmo tempo em que fornece uma base sólida para futuras expansões e melhorias na aplicação.

Para mais informações sobre o TinyDB, consulte a [documentação oficial do TinyDB](https://tinydb.readthedocs.io/en/latest/).