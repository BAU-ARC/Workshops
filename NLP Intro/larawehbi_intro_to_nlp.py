import numpy as np
import nltk
# nltk.download('stopwords')
# nltk.download('punkt')

from nltk.corpus import stopwords

corpus = 'This is a rainy day '
corpus = 'This is a rainy day and it looks cold. I think it will be cozy'
stop_words = set(stopwords.words('english'))
# {'itself', 'why', 'yourselves', 'under', 'doesn', "wasn't", 'on', 'wasn', 'just', 'a', 't', 'yourself', 'all', 'does', "mightn't", 'him', 'when', 'her', "didn't", 'they', 'won', 'has', 'more', 'y', 'me', 'own', 'too', 'you', 'shan', 'it', 'during', 'down', 'm', 'weren', 'wouldn', 'before', 'this', 'each', 'for', 'to', "shouldn't", 'my', 'now', "isn't", 'here', 'an', 'whom', 'after', 'below', 're', 'couldn', 'if', 'in', 'between', "needn't", 'don', 'out', 'he', 'few', 'while', 'be', 'not', 'shouldn', 'as', 'haven', 'further', 'nor', 'ain', 'mustn', 'is', 'your', 'them', 'both', 'needn', 'we', 'from', 'there', 'ma', 'have', 'with', 'so', "aren't", "couldn't", 'ours', 'aren', 'what', 'was', 'because', "you'll", 'themselves', "should've", 'such', 'were', 'being', "you've", 'ourselves', 'but', "you'd", 'no', 'am', "wouldn't", 'herself', 'having', 'who', 'o', 'hasn', 's', 'isn', 'theirs', 'until', 'she', 'again', 'once', 'doing', 'or', 'very', 'i', "won't", 'these', 'at', 'll', 'had', 've', 'up', 'same', 'most', "weren't", 'd', "don't", 'mightn', 'should', 'and', 'myself', 'into', "mustn't", 'its', "that'll", 'those', 'of', "shan't", "hadn't", 'been', 'hadn', 'some', 'that', 'his', "haven't", 'their', 'are', 'only', 'over', 'do', 'didn', 'then', "hasn't", 'hers', 'can', 'which', 'our', "doesn't", 'yours', 'himself', 'where', 'than', 'about', 'through', "it's", 'did', 'against', 'above', 'by', 'any', 'other', "you're", 'off', 'how', 'the', "she's", 'will'}
sents = nltk.sent_tokenize(corpus) # Tokenizing

words = nltk.word_tokenize(corpus) # Splitting into individual words

final_words = [] 
for x  in words:
    if x not in stop_words: 
        final_words.append(x) # remove stop words

from nltk.stem import SnowballStemmer

ss = SnowballStemmer('english') # Stemming
stemmed_words = [ss.stem(x) for x in final_words]