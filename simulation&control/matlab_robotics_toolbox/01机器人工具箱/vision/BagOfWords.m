%BagOfWords Bag of words class
%
% The BagOfWords class holds sets of features for a number of images and 
% supports image retrieval by comparing new images with those in the 'bag'.
%
% Methods::
% isword        Return all features assigned to word
% occurrences   Return number of occurrences of word
% remove_stop   Remove stop words
% wordvector    Return word frequency vector
% wordfreq      Return words and their frequencies
% similarity    Compare two word bags
% contains      List the images that contain a word
% exemplars     Display examples of word support regions
% display       Display the parameters of the bag of words
% char          Convert the parameters of the bag of words to a string
%
% Properties::
% K         The number of clusters specified
% nstop     The number of stop words specified
% nimages   The number of images in the bag
%
% Reference::
% 
% J.Sivic and A.Zisserman,
% "Video Google: a text retrieval approach to object matching in videos",
% in Proc. Ninth IEEE Int. Conf. on Computer Vision, pp.1470-1477, Oct. 2003.
%
% See also PointFeature.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

classdef BagOfWords < handle
    properties
        features    % vector of all features in the bag (PointFeature class)

        K       % number of clusters
        C       % cluster centres (NDxNW)
        words   % vector of word indices (NW)

        nstop     % number of stop words
        stopwords  % list of stop words
        map        % maps word index with stop words to word index without stop words

        nimages       % number of images (NI)

        wv          % cached word vectors
    end

    methods
        function bag = BagOfWords(sf, a1)
        %BagOfWords.BagOfWords Create a BagOfWords object
        %
        % B = BagOfWords(F, K) is a new bag of words created from the feature
        % vector F and with K words.  F can also be a cell array, as produced 
        % by ISURF() for an image sequence.
        %
        % The features are sorted into K clusters and each cluster is termed
        % a visual word.
        %
        % B = BagOfWords(F, B2) is a new bag of words created from the feature
        % vector F but clustered to the words (and stop words) from the existing
        % bag B2.
        %
        % Notes::
        % - Uses the MEX function vl_kmeans to perform clustering (vlfeat.org).
        %
        % See also PointFeature, ISURF.

            % save the feature vector
            if iscell(sf)
                bag.features = [sf{:}];
            else
                bag.features = sf;
            end
            bag.nimages = max([bag.features.image_id]);

            if isnumeric(a1)
                K = a1;
                % do the clustering
                [bag.C,L] = vl_kmeans([bag.features.descriptor], K, ...
                    'verbose', 'algorithm', 'elkan');

                bag.K = K;
                bag.words = double(L);

            elseif isa(a1, 'BagOfWords')
                oldbag = a1;

                bag.K = oldbag.K;
                bag.stopwords = oldbag.stopwords;

                % cluster using number of words from old bag
                bag.words = closest([bag.features.descriptor], oldbag.C);

                k = find(ismember(bag.words, oldbag.stopwords));

                fprintf('Removing %d features associated with stop words\n', length(k));

                bag.words(k) = [];
                bag.words = oldbag.map(bag.words);
                bag.features(k) = [];

                bag.compute_wv(oldbag);

            end
        end

        function f = isword(bag, words)
        %BagOfWords.isword Features from words
        %
        % F = B.isword(W) is a vector of feature objects that are assigned to any of
        % the word W.  If W is a vector of words the result is a vector of features
        % assigned to all the words in W.
            k = ismember(bag.words, words);
            f = bag.features(k);
        end

        function n = occurrence(bag, word)
        %BagOfWords.occurrence Word occurrence
        %
        % N = B.occurrence(W) is the number of occurrences of the word W across
        % all features in the bag.
            n = sum(bag.words == word);
        end

        function [all2, S] = remove_stop(bag, nstop)
        %BagOfWords.remove_stop Remove stop words
        %
        % B.remove_stop(N) removes the N most frequent words (the stop words)
        % from the bag.  All remaining words are renumbered so that the word
        % labels are consecutive.

            [w,f] = count_unique(bag.words);
            [f,i] = sort(f, 'descend');
            bag.stopwords = w(i(1:nstop));

            % remove all features that are stop words from L and all
            k = find(ismember(bag.words, bag.stopwords));

            fprintf('Removing %d features associated with %d most frequent words\n', ...
                length(k), nstop);

            % fix the labels
            b = zeros(1,length(bag.words));
            b(bag.stopwords) = 1;
            bag.map = [1:length(bag.words)] - cumsum(b);

            bag.words(k) = [];
            bag.words = bag.map(bag.words);
            bag.features(k) = [];

        end

        function wv = wordvector(bag, k)
        %BagOfWords.wordvector  Word frequency vector
        %
        % WF = B.wordvector(J) is the word frequency vector for the J'th image
        % in the bag.  The vector is Kx1 and the angle between any two WFVs is
        % an indication of image similarity.
        %
        % Notes::
        % - The word vector is expensive to compute so a lazy evaluation is
        %   performed on the first call to this function
            if isempty(bag.wv)
                bag.compute_wv();
            end
            if nargin > 1
                wv = bag.wv(:,k);
            else
                wv = bag.wv;
            end
        end

        % compute  image-word frequency
        function W = iwf(bag)

            N = bag.nimages;  % number of images

            % Create the word frequency matrix W
            %  column correspond to images
            %  row correspond to words
            %  each element is the number of occurences of that word in that iamge
            W = [];
            id = [bag.features.image_id];

            nl = bag.K - length(bag.stopwords);

            for i=1:bag.nimages
                % get the words associated with image i
                words = bag.words(id == i);

                % create columns of the W
                [w,f] = count_unique(words);
                v = zeros(nl,1);
                v(w) = f;
                W = [W v];
            end
        end

        function W = compute_wv(bag, bag2)

            if nargin == 2
                Wv = bag2.iwf();
                N = bag2.nimages;
                W = bag.iwf();
            else
                Wv = bag.iwf();
                N = bag.nimages;
                W = Wv;
            end

            Ni = sum( Wv'>0 );

            m = [];
            for i=1:bag.nimages
                % number of words in this image
                nd = sum( W(:,i) );

                % word occurrence frequency
                nid = W(:,i)';

                v = nid/nd .* log(N./Ni);
                m = [m v'];
            end

            if nargout == 1
                W = m;
            else
                bag.wv = m;
            end
        end

        function [w,f] = wordfreq(bag)
        %BagOfWords.wordfreq Word frequency statistics
        %
        % [W,N] = B.wordfreq() is a vector of word labels W and the corresponding
        % elements of N are the number of occurrences of that word.
            [w,f] = count_unique(bag.words);
        end

        % compute similarity matrix
        function sim = similarity(bag1, bag2)
            wv1 = bag1.wordvector;
            wv2 = bag2.wordvector;
            for i=1:bag1.nimages
                for j=1:bag2.nimages
                    v1 = wv1(:,i); v2 = wv2(:,j);
                    sim(i,j) = dot(v1,v2) / (norm(v1) * norm(v2));
                end
            end
        end

        function display(bag)
        %BagOfWords.display Display value
        %
        % B.display() displays the parameters of the bag in a compact human
        % readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a BagOfWords object and the command has no trailing
        %   semicolon.
        %
        % See also BagOfWords.char.

            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(bag))
            if loose
                disp(' ');
            end
        end

        function s = char(bag)
        %BagOfWords.char Convert to string
        %
        % S = B.char() is a compact string representation of a bag of words.
            s = sprintf(...
            'BagOfWords: %d features from %d images\n           %d words, %d stop words\n', ...
                length(bag.features), bag.nimages, ...
                bag.K-length(bag.stopwords), length(bag.stopwords));
        end


        function v = contains(bag, word)
        %BagOfWords.contains Find images containing word
        %
        % K = B.contains(W) is a vector of the indices of images in the sequence that
        % contain one or more instances of the word W.
            v = unique([bag.isword(word).image_id]);
        end
            
        function exemplars(bag, words, images, varargin)
        %BagOfWords.exemplars Display exemplars of words
        %
        % B.exemplars(W, IMAGES, OPTIONS) displays examples of the support regions of
        % the words specified by the vector W.  The examples are displayed as a table
        % of thumbnail images.  The original sequence of images from which the features
        % were extracted must be provided as IMAGES.
        %
        % Options::
        % 'ncolumns',N      Number of columns to display (default 10)
        % 'maxperimage',M   Maximum number of exemplars to display from any 
        %                   one image (default 2)
        % 'width',W         Width of each thumbnail [pixels] (default 50)

            nwords = length(words);
            gap = 2;

            opt.ncolumns = 10;
            opt.maxperimage = 2;
            opt.width = 50;

            opt = tb_optparse(opt, varargin);

            % figure the number of exemplars to show, no more than opt.maxperimage
            % from any one image
            nexemplars = 0;
            for w=words
                h = hist(bag.contains(w));
                h = min(h, opt.maxperimage);
                nexemplars = nexemplars + sum(h);
            end
            opt.ncolumns = min(nexemplars, opt.ncolumns);

            Ng = opt.width+gap;
            panel = zeros(nwords*Ng, opt.ncolumns*Ng);
            L = bag.words;

            for i=1:nwords
                % for each word specified
                word = words(i);

                features = bag.isword(word);  % find features corresponding to the word

                image_prev = [];
                count = 1;
                for j=1:length(features)
                    % for each instance of that word
                    sf = features(j);

                    % only display one template from each image to show some
                    % variety
                    if sf.image_id == image_prev
                        c = c - 1;
                        if c <= 0
                            continue;
                        end
                    else
                        c = opt.maxperimage;
                    end
                    % extract it from the containing image
                    out = sf.support(images, opt.width);

                    % paste it into the panel
                    panel = ipaste(panel, out, [count-1 i-1]*Ng, 'zero');
                    image_prev = sf.image_id;
                    count = count + 1;
                    if count > opt.ncolumns
                        break;
                    end
                end
            end

            idisp(panel, 'plain');


            for i=1:nwords
                % for each word specified
                word = words(i);
                features = bag.isword(word);  % find features corresponding to the word

                image_prev = [];
                count = 1;
                for j=1:length(features)
                    % for each instance of that word
                    sf = features(j);
                    if sf.image_id == image_prev
                        c = c - 1;
                        if c <= 0
                            continue;
                        end
                    else
                        c = opt.maxperimage;
                    end
                    % extract it from the containing image
                    text((count-1)*Ng+gap*2, (i-1)*Ng+3*gap, ...
                        sprintf('%d/%d', word, sf.image_id), 'Color', 'g')
                    image_prev = sf.image_id;
                    count = count + 1;
                    if count > opt.ncolumns
                        break;
                    end
                end
            end
        end
    end
end
