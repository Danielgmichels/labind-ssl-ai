# Usa uma imagem oficial do Python, versão leve (slim)
FROM python:3.10-slim

# Define o diretório de trabalho dentro do container
WORKDIR /app

# Copia o arquivo de dependências primeiro (otimiza o cache do Docker)
COPY requirements.txt .

# Instala as dependências
RUN pip install --no-cache-dir -r requirements.txt

# Copia todo o resto do seu código para dentro do container
COPY . .

# Comando que será executado quando o container ligar
CMD ["python", "main.py"]